// SPDX-License-Identifier: MIT
//! The FM-1's USB DRIVE item (rust/src/usb_drive.rs, and `usb_stage2` in sdboot.rs) against a simulated USB0
//! controller in host mode, with a simulated flash drive on it holding generated FAT32 images.
//!
//! The controller is modelled as the driver uses it: its own registers behind CON1 (FADDR, POWER, INDEX, DEVCTL,
//! endpoint 0's CSR0 and endpoint 1's TXCSR1 and RXCSR1, their counts and types), and the DMA buffers whose addresses
//! it's given. The drive answers control requests on endpoint 0 at the address it was given, and SCSI commands
//! through the bulk-only transport on endpoints 1 IN and 2 OUT. It can be made to fail: absent, not a drive, sectors
//! of another size, slow to be ready, refusing reads, or pulled out partway.
//!
//! What this checks is the driver's logic against the protocol as the model has it, not the hardware: timing, the
//! controller's quirks and real drives' firmware are for a unit.
use crate::Global;
use crate::formats::crc16;
use crate::hw::{Text, bit};
use crate::sdboot::usb_stage2;
use core::ffi::{c_int, c_void};
use std::collections::VecDeque;

// The registers (rust/src/usb_drive.rs).
const CLK_CON1: usize = 0x10010;
const TIMER: usize = 0x10400;
const TIMER_COUNT: usize = 0x10404;
const TIMER_PERIOD: usize = 0x10408;
const CON0: usize = 0x11800;
const CON1: usize = 0x11804;
const EP0_CNT: usize = 0x11808;
const EP1_CNT: usize = 0x1180c;
const EP0_ADR: usize = 0x11818;
const EP1_TADR: usize = 0x1181c;
const EP1_RADR: usize = 0x11820;
const IO_CON0: usize = 0x51000;
const IO_CON1: usize = 0x51004;
const REGISTERS: [usize; 13] = [
    CLK_CON1,
    TIMER,
    TIMER_COUNT,
    TIMER_PERIOD,
    CON0,
    CON1,
    EP0_CNT,
    EP1_CNT,
    EP0_ADR,
    EP1_TADR,
    EP1_RADR,
    IO_CON0,
    IO_CON1,
];
// The controller's own registers.
const FADDR: u32 = 0;
const POWER: u32 = 1;
const INDEX: u32 = 14;
const DEVCTL: u32 = 15;
const CSR: u32 = 17;
const RXCSR1: u32 = 20;
const COUNT: u32 = 22;
const RXCOUNT2: u32 = 23;
const TXTYPE: u32 = 24;
const RXTYPE: u32 = 26;
/// Timer counts that pass at each read of the count: 0.1 ms at the 48 MHz the test runs the item at.
const TICKS_PER_READ: u32 = 4800;
const LSB_MHZ: u32 = 48;
const WAIT_MS: u32 = 3000;

// Where USB.BIN loads and the most it may be (docs/SPEC.md §11.5), and where the item keeps its buffers: the test's
// own copies, so that a wrong value in the crate fails here.
const STAGE2_ADDRESS: usize = 0x01c2_8000;
const STAGE2_MAX: usize = 0x8000;
const BUFFERS: usize = 0x01c1_6800;
/// What the test maps: the buffers up to the end of stage 2's RAM.
const MAP_START: usize = 0x01c1_6000;
const MAP_LEN: usize = STAGE2_ADDRESS + STAGE2_MAX - MAP_START;

// What <sys/mman.h> has, on Linux.
const PROT_READ: c_int = 1;
const PROT_WRITE: c_int = 2;
const MAP_PRIVATE: c_int = 2;
const MAP_ANONYMOUS: c_int = 0x20;
/// At that address or not at all: nothing that is already there is replaced.
const MAP_FIXED_NOREPLACE: c_int = 0x10_0000;

unsafe extern "C" {
    fn mmap(
        addr: *mut c_void,
        length: usize,
        prot: c_int,
        flags: c_int,
        fd: c_int,
        offset: i64,
    ) -> *mut c_void;
}

// The drive's image.
const SECTORS: u32 = 4096;
const RESERVED: u32 = 4;
const FAT_SECTORS: u32 = 8;
const SPC: u32 = 2;
const CLUSTER: usize = (SPC * 512) as usize;
const PARTITION: u32 = 8;

/// The flash drive: its USB state, its bulk-only transport, and its image.
struct Drive {
    attached: bool,
    /// Not a mass-storage device: its configuration has no bulk endpoints.
    no_bulk: bool,
    /// Endpoint 0's packet size.
    mps0: usize,
    /// The size of its sectors, as READ CAPACITY gives it.
    block: u32,
    address: u32,
    /// SET_ADDRESS takes effect once its status stage is done.
    pending_address: Option<u32>,
    configured: bool,
    /// What a control transfer's IN data stage sends.
    ep0_in: VecDeque<u8>,
    halted_in: bool,
    halted_out: bool,
    /// The data of the command in progress, then its status.
    data_in: VecDeque<u8>,
    csw: Option<[u8; 13]>,
    /// TEST UNIT READY answers "not ready" this many times first.
    not_ready: u32,
    /// The READs it refuses, stalling the data stage, once `refuse_after` have succeeded.
    refuse_reads: u32,
    refuse_after: u32,
    /// It's pulled out once this many READs have succeeded (0: never).
    pull_after_reads: u32,
    reads: u32,
    /// What it was asked, for the tests to check: SCSI opcodes, and the bus resets and bulk-only resets.
    commands: Vec<u8>,
    bus_resets: u32,
    bot_resets: u32,
    disk: Vec<u8>,
}

/// USB0 in host mode, with the drive on it.
struct Usb {
    slots: [u32; REGISTERS.len()],
    sie: [u32; 32],
    session: bool,
    csr0: u32,
    txcsr1: u32,
    rxcsr1: u32,
    count0: u32,
    rx_count: u32,
    drive: Drive,
}

static USB: Global<Option<Usb>> = Global::new(None);

fn usb() -> &'static mut Usb {
    // Safety: one thread, and no reference to it is kept over a call that takes another.
    unsafe { USB.get() }.as_mut().unwrap()
}

fn slot_index(address: usize) -> usize {
    REGISTERS
        .iter()
        .position(|&r| r == address)
        .unwrap_or_else(|| panic!("register {address:#x}"))
}

impl Usb {
    fn slot(&mut self, address: usize) -> &mut u32 {
        &mut self.slots[slot_index(address)]
    }

    /// Whether the drive hears what's sent to `address`: attached, the bus running and the address its own.
    fn reaches(&self, address: u32) -> bool {
        self.drive.attached
            && self.session
            && self.slots[slot_index(CON0)] & 4 != 0
            && address == self.drive.address
    }

    /// What the driver wrote since the last access, then what the next one reads.
    fn flush(&mut self) {
        // A command for the controller: written with bit 15 clear, which it rests with set.
        let c = *self.slot(CON1);
        if c & bit(15) == 0 {
            let r = c >> 8 & 0x3f;
            let value = if c & 0x4000 != 0 {
                self.sie_read(r)
            } else {
                self.sie_write(r, c & 0xff);
                0
            };
            *self.slot(CON1) = bit(15) | value;
        }
        // D+ pulled up while a drive is attached.
        let attached = self.drive.attached;
        let con0 = self.slot(CON0);
        *con0 = if attached {
            *con0 | bit(17)
        } else {
            *con0 & !(bit(17) | bit(16))
        };
    }

    fn sie_read(&mut self, r: u32) -> u32 {
        let index = self.sie[INDEX as usize];
        match r {
            DEVCTL => {
                if self.session && self.drive.attached {
                    0x44
                } else {
                    0
                }
            }
            CSR => {
                if index == 0 {
                    self.csr0
                } else {
                    self.txcsr1
                }
            }
            RXCSR1 => self.rxcsr1,
            COUNT => {
                if index == 0 {
                    self.count0
                } else {
                    self.rx_count & 0xff
                }
            }
            RXCOUNT2 => self.rx_count >> 8,
            _ => self.sie[r as usize],
        }
    }

    fn sie_write(&mut self, r: u32, v: u32) {
        let index = self.sie[INDEX as usize];
        match r {
            POWER => {
                let was = self.sie[POWER as usize];
                self.sie[POWER as usize] = v;
                if was & 8 != 0 && v & 8 == 0 {
                    self.drive.bus_reset();
                }
            }
            DEVCTL => self.session = v & 1 != 0,
            CSR if index == 0 => self.csr0_write(v),
            CSR => self.txcsr1_write(v),
            RXCSR1 => self.rxcsr1_write(v),
            _ => self.sie[r as usize] = v,
        }
    }

    fn buffer(&mut self, register: usize, n: usize) -> &'static mut [u8] {
        let at = *self.slot(register) as usize;
        assert!(
            at >= BUFFERS && at + n <= STAGE2_ADDRESS,
            "a DMA buffer at {at:#x}"
        );
        // Safety: mapped by the test, and the driver isn't using it while the controller is.
        unsafe { core::slice::from_raw_parts_mut(at as *mut u8, n) }
    }

    fn csr0_write(&mut self, v: u32) {
        let address = self.sie[FADDR as usize];
        self.csr0 = match v {
            0 => 0,
            // SETUP
            0x0a => {
                assert!(*self.slot(EP0_CNT) == 8);
                if !self.reaches(address) {
                    0x10
                } else {
                    let setup: [u8; 8] = self.buffer(EP0_ADR, 8).try_into().unwrap();
                    if self.drive.setup(setup) { 0 } else { 0x04 }
                }
            }
            // IN data
            0x20 => {
                if !self.reaches(address) {
                    0x10
                } else {
                    let n = self.drive.ep0_in.len().min(self.drive.mps0);
                    let packet: Vec<u8> = self.drive.ep0_in.drain(..n).collect();
                    self.buffer(EP0_ADR, n).copy_from_slice(&packet);
                    self.count0 = n as u32;
                    0x01
                }
            }
            // status OUT, after IN data
            0x42 => {
                assert!(*self.slot(EP0_CNT) == 0);
                if self.reaches(address) { 0 } else { 0x10 }
            }
            // status IN, after no data
            0x60 => {
                if !self.reaches(address) {
                    0x10
                } else {
                    if let Some(a) = self.drive.pending_address.take() {
                        self.drive.address = a;
                    }
                    self.count0 = 0;
                    0x01
                }
            }
            _ => panic!("CSR0 written {v:#x}: a control transfer the driver doesn't make"),
        };
    }

    fn txcsr1_write(&mut self, v: u32) {
        let address = self.sie[FADDR as usize];
        if v & 1 == 0 {
            // clearing a stall or an error, a flush, or the toggle back to DATA0
            self.txcsr1 = 0;
            return;
        }
        let n = *self.slot(EP1_CNT) as usize;
        let endpoint = self.sie[TXTYPE as usize] & 15;
        assert!(self.sie[TXTYPE as usize] & 0x30 == 0x20, "bulk");
        self.txcsr1 = if !self.reaches(address) {
            // three errors
            0x04
        } else if endpoint != 2 || self.drive.halted_out || !self.drive.configured {
            0x20
        } else {
            let packet = self.buffer(EP1_TADR, n).to_vec();
            self.drive.bulk_out(&packet);
            0
        };
    }

    fn rxcsr1_write(&mut self, v: u32) {
        let address = self.sie[FADDR as usize];
        if v & 0x20 == 0 {
            // releasing the buffer, clearing a stall, or the toggle back to DATA0
            self.rxcsr1 = 0;
            return;
        }
        let endpoint = self.sie[RXTYPE as usize] & 15;
        assert!(self.sie[RXTYPE as usize] & 0x30 == 0x20, "bulk");
        self.rxcsr1 = if !self.reaches(address) {
            0x04
        } else if endpoint != 1 || self.drive.halted_in || !self.drive.configured {
            0x40
        } else {
            match self.drive.bulk_in(64) {
                Some(packet) => {
                    // The controller's DMA writes a packet; the driver gives it 72 bytes.
                    self.buffer(EP1_RADR, 72)[..packet.len()].copy_from_slice(&packet);
                    self.rx_count = packet.len() as u32;
                    0x01
                }
                // NAK, which the controller keeps sending: nothing comes
                None => 0,
            }
        };
    }
}

impl Drive {
    fn new(disk: Vec<u8>) -> Drive {
        Drive {
            attached: true,
            no_bulk: false,
            mps0: 64,
            block: 512,
            address: 0,
            pending_address: None,
            configured: false,
            ep0_in: VecDeque::new(),
            halted_in: false,
            halted_out: false,
            data_in: VecDeque::new(),
            csw: None,
            not_ready: 0,
            refuse_reads: 0,
            refuse_after: 0,
            pull_after_reads: 0,
            reads: 0,
            commands: Vec::new(),
            bus_resets: 0,
            bot_resets: 0,
            disk,
        }
    }

    fn bus_reset(&mut self) {
        self.address = 0;
        self.pending_address = None;
        self.configured = false;
        self.halted_in = false;
        self.halted_out = false;
        self.data_in.clear();
        self.csw = None;
        self.bus_resets += 1;
    }

    fn descriptor(&self, kind: u32) -> Option<Vec<u8>> {
        match kind {
            1 => Some(vec![
                18,
                1,
                0,
                2,
                0,
                0,
                0,
                self.mps0 as u8,
                0x81,
                0x07,
                0x81,
                0x55,
                0,
                1,
                1,
                2,
                3,
                1,
            ]),
            2 if self.no_bulk => {
                // A keyboard: one interrupt IN endpoint.
                let mut d = vec![9, 2, 34, 0, 1, 1, 0, 0x80, 50];
                d.extend_from_slice(&[9, 4, 0, 0, 1, 3, 1, 1, 0]);
                d.extend_from_slice(&[9, 0x21, 0x11, 1, 0, 1, 0x22, 63, 0]);
                d.extend_from_slice(&[7, 5, 0x81, 3, 8, 0, 10]);
                Some(d)
            }
            2 => {
                let mut d = vec![9, 2, 32, 0, 1, 1, 0, 0x80, 50];
                // mass storage, SCSI, bulk-only
                d.extend_from_slice(&[9, 4, 0, 0, 2, 8, 6, 0x50, 0]);
                d.extend_from_slice(&[7, 5, 0x81, 2, 64, 0, 0]);
                d.extend_from_slice(&[7, 5, 0x02, 2, 64, 0, 0]);
                Some(d)
            }
            _ => None,
        }
    }

    /// A SETUP packet: false to stall it.
    fn setup(&mut self, s: [u8; 8]) -> bool {
        let value = u16::from_le_bytes([s[2], s[3]]) as u32;
        let index = u16::from_le_bytes([s[4], s[5]]) as u32;
        let length = u16::from_le_bytes([s[6], s[7]]) as usize;
        self.ep0_in.clear();
        match (s[0], s[1]) {
            (0x80, 6) => match self.descriptor(value >> 8) {
                Some(d) => {
                    self.ep0_in.extend(&d[..d.len().min(length)]);
                    true
                }
                None => false,
            },
            (0x00, 5) if length == 0 => {
                self.pending_address = Some(value);
                true
            }
            (0x00, 9) if length == 0 && self.address != 0 => {
                self.configured = value == 1;
                value == 1
            }
            (0x02, 1) if length == 0 && value == 0 => {
                match index {
                    0x81 => self.halted_in = false,
                    0x02 => self.halted_out = false,
                    _ => return false,
                }
                true
            }
            (0x21, 0xff) if length == 0 && index == 0 => {
                self.data_in.clear();
                self.csw = None;
                self.bot_resets += 1;
                true
            }
            _ => false,
        }
    }

    fn sectors(&self) -> u32 {
        (self.disk.len() / 512) as u32
    }

    /// A command block on endpoint 2.
    fn bulk_out(&mut self, cbw: &[u8]) {
        assert!(cbw.len() == 31 && cbw[..4] == *b"USBC", "a command block");
        assert!(
            self.csw.is_none() && self.data_in.is_empty(),
            "a command before the last one's status was read"
        );
        let tag: [u8; 4] = cbw[4..8].try_into().unwrap();
        let length = u32::from_le_bytes(cbw[8..12].try_into().unwrap()) as usize;
        let cdb = &cbw[15..15 + cbw[14] as usize];
        assert!(length == 0 || cbw[12] & 0x80 != 0, "only IN data");
        self.commands.push(cdb[0]);
        let mut data = Vec::new();
        let mut status = 0;
        match (cdb[0], cdb.len()) {
            (0x00, 6) => {
                if self.not_ready != 0 {
                    self.not_ready -= 1;
                    status = 1;
                }
            }
            (0x03, 6) => {
                data = vec![0; 18];
                data[0] = 0x70;
                data[2] = 2; // NOT READY
                data[7] = 10;
                data[12] = 0x3a; // medium not present
            }
            (0x25, 10) => {
                data.extend_from_slice(&(self.sectors() - 1).to_be_bytes());
                data.extend_from_slice(&self.block.to_be_bytes());
            }
            (0x28, 10) => {
                let lba = u32::from_be_bytes(cdb[2..6].try_into().unwrap());
                let n = u16::from_be_bytes([cdb[7], cdb[8]]) as u32;
                assert!(n == 1, "one sector a READ");
                if (self.refuse_reads != 0 && self.reads >= self.refuse_after)
                    || lba + n > self.sectors()
                {
                    self.refuse_reads = self.refuse_reads.saturating_sub(1);
                    status = 1;
                    self.halted_in = true;
                } else {
                    data.extend_from_slice(&self.disk[lba as usize * 512..][..512]);
                    self.reads += 1;
                }
            }
            _ => {
                status = 1;
                if length != 0 {
                    self.halted_in = true;
                }
            }
        }
        data.truncate(length);
        let residue = (length - data.len()) as u32;
        self.data_in.extend(&data);
        let mut csw = [0; 13];
        csw[..4].copy_from_slice(b"USBS");
        csw[4..8].copy_from_slice(&tag);
        csw[8..12].copy_from_slice(&residue.to_le_bytes());
        csw[12] = status;
        self.csw = Some(csw);
    }

    /// An IN packet on endpoint 1, or None for a NAK.
    fn bulk_in(&mut self, mps: usize) -> Option<Vec<u8>> {
        if !self.data_in.is_empty() {
            let n = self.data_in.len().min(mps);
            return Some(self.data_in.drain(..n).collect());
        }
        let csw = self.csw.take()?;
        // Pulled out as the status of the READ that reached the count was about to go.
        if self.pull_after_reads != 0 && self.reads >= self.pull_after_reads {
            self.attached = false;
            return None;
        }
        Some(csw.to_vec())
    }
}

/// The chip's registers, with the controller and the drive behind them.
pub fn platform_register(address: usize) -> *mut u32 {
    let usb = usb();
    usb.flush();
    if address == TIMER_COUNT {
        let count = usb.slot(TIMER_COUNT);
        *count = count.wrapping_add(TICKS_PER_READ);
    }
    usb.slot(address)
}

// What the crate takes from stage 0 by name in this build (`import_early`).

/// The spacing between the controller's accesses (usb_midi.rs): no time on the simulated chip.
#[unsafe(no_mangle)]
pub extern "C" fn settle(_n: u32) {}

// A FAT32 image, as on a drive: base 0 has no partition table.

struct Image {
    disk: Vec<u8>,
    base: u32,
    next_free: u32,
}

impl Image {
    fn new(base: u32) -> Image {
        let mut disk = vec![0; (SECTORS * 512) as usize];
        if base != 0 {
            disk[454..458].copy_from_slice(&base.to_le_bytes());
            disk[510..512].copy_from_slice(&[0x55, 0xaa]);
        }
        let boot = &mut disk[base as usize * 512..];
        boot[11..13].copy_from_slice(&512u16.to_le_bytes());
        boot[13] = SPC as u8;
        boot[14..16].copy_from_slice(&(RESERVED as u16).to_le_bytes());
        boot[16] = 2;
        boot[36..40].copy_from_slice(&FAT_SECTORS.to_le_bytes());
        boot[44..48].copy_from_slice(&2u32.to_le_bytes());
        boot[510..512].copy_from_slice(&[0x55, 0xaa]);
        let mut image = Image {
            disk,
            base,
            next_free: 2,
        };
        image.allocate(0);
        image
    }

    fn set_fat(&mut self, c: u32, v: u32) {
        let at = ((self.base + RESERVED) * 512 + c * 4) as usize;
        self.disk[at..at + 4].copy_from_slice(&v.to_le_bytes());
    }

    fn cluster(&mut self, c: u32) -> &mut [u8] {
        let data = self.base + RESERVED + 2 * FAT_SECTORS;
        &mut self.disk[((data + (c - 2) * SPC) * 512) as usize..][..CLUSTER]
    }

    fn allocate(&mut self, after: u32) -> u32 {
        let c = self.next_free;
        self.next_free += 1;
        self.set_fat(c, 0x0fff_ffff);
        if after != 0 {
            self.set_fat(after, c);
        }
        c
    }

    fn entry(&mut self, dir: u32, index: usize, name: &[u8; 11], attr: u8, start: u32, size: u32) {
        let e = &mut self.cluster(dir)[index * 32..][..32];
        e[..11].copy_from_slice(name);
        e[11] = attr;
        e[20..22].copy_from_slice(&((start >> 16) as u16).to_le_bytes());
        e[26..28].copy_from_slice(&(start as u16).to_le_bytes());
        e[28..32].copy_from_slice(&size.to_le_bytes());
    }

    fn folder(&mut self, parent: u32, index: usize, name: &[u8; 11]) -> u32 {
        let c = self.allocate(0);
        self.entry(parent, index, name, 0x10, c, 0);
        c
    }

    fn file(&mut self, dir: u32, index: usize, name: &[u8; 11], bytes: &[u8]) {
        let mut first = 0;
        let mut c = 0;
        for chunk in bytes.chunks(CLUSTER) {
            c = self.allocate(c);
            if first == 0 {
                first = c;
            }
            self.cluster(c)[..chunk.len()].copy_from_slice(chunk);
        }
        self.entry(dir, index, name, 0x20, first, bytes.len() as u32);
    }
}

/// A stage 2 image of `size` bytes, with its trailer.
fn stage2(size: usize) -> Vec<u8> {
    let mut out: Vec<u8> = (0..size - 8).map(|i| (i * 7 + 3) as u8).collect();
    let crc = crc16(0, &out);
    out.extend_from_slice(b"STG2");
    out.extend_from_slice(&crc.to_le_bytes());
    out.extend_from_slice(&[0, 0]);
    out
}

/// A drive with /MBOOT/USB.BIN on it.
fn drive_with(base: u32, usb_bin: &[u8]) -> Drive {
    let mut image = Image::new(base);
    let mboot = image.folder(2, 0, b"MBOOT      ");
    image.file(mboot, 0, b"MENU    BIN", &stage2(600));
    image.file(mboot, 1, b"USB     BIN", usb_bin);
    Drive::new(image.disk)
}

fn ram() -> &'static mut [u8] {
    // Safety: mapped for the test, which has one thread and keeps no other reference to it.
    unsafe { core::slice::from_raw_parts_mut(STAGE2_ADDRESS as *mut u8, STAGE2_MAX) }
}

/// Chooses the item with `drive` on the port: what it showed, if it didn't run USB.BIN.
fn run(drive: Drive) -> Option<Text> {
    ram().fill(0);
    // Safety: as for `usb`.
    *unsafe { USB.get() } = Some(Usb {
        slots: [0; REGISTERS.len()],
        sie: [0; 32],
        session: false,
        csr0: 0,
        txcsr1: 0,
        rxcsr1: 0,
        count0: 0,
        rx_count: 0,
        drive,
    });
    *usb().slot(CON1) = bit(15);
    let r = usb_stage2(LSB_MHZ, WAIT_MS);
    // The controller is left off, for stage 2 or the menu to start again.
    assert!(*usb().slot(CON0) & 0xff == 0 && *usb().slot(IO_CON0) == 0);
    r
}

fn says(r: Option<Text>, text: &str) -> bool {
    r.is_some_and(|r| r.as_bytes() == text.as_bytes())
}

#[test]
fn usb_drive() {
    // Safety: a new mapping, of memory that nothing has.
    let mapped = unsafe {
        mmap(
            MAP_START as *mut c_void,
            MAP_LEN,
            PROT_READ | PROT_WRITE,
            MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE,
            -1,
            0,
        )
    };
    assert!(mapped == MAP_START as *mut c_void);
    let usb_bin = stage2(19_064);

    // In a partition, and without a partition table: loaded, and USB.BIN, not MENU.BIN.
    assert!(run(drive_with(PARTITION, &usb_bin)).is_none());
    assert!(ram()[..usb_bin.len()] == usb_bin[..]);
    let d = &usb().drive;
    assert!(d.configured && d.address == 1 && d.bus_resets == 2 && d.bot_resets == 0);
    assert!(d.commands[..2] == [0x00, 0x25] && d.commands[2..].iter().all(|&c| c == 0x28));
    assert!(run(drive_with(0, &usb_bin)).is_none() && ram()[..usb_bin.len()] == usb_bin[..]);
    // The biggest USB.BIN there may be, and one that ends partway through a sector.
    let biggest = stage2(STAGE2_MAX);
    assert!(run(drive_with(PARTITION, &biggest)).is_none() && ram()[..] == biggest[..]);
    let odd = stage2(1000);
    assert!(run(drive_with(PARTITION, &odd)).is_none() && ram()[..odd.len()] == odd[..]);

    // Endpoint 0 of 8 bytes: descriptors in several packets.
    let mut drive = drive_with(PARTITION, &usb_bin);
    drive.mps0 = 8;
    assert!(run(drive).is_none() && ram()[..usb_bin.len()] == usb_bin[..]);

    // Slow to be ready: asked what's wrong after each refusal, then read.
    let mut drive = drive_with(PARTITION, &usb_bin);
    drive.not_ready = 3;
    assert!(run(drive).is_none() && ram()[..usb_bin.len()] == usb_bin[..]);
    let c = &usb().drive.commands;
    assert!(c[..8] == [0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x25]);
    // Never ready.
    let mut drive = drive_with(PARTITION, &usb_bin);
    drive.not_ready = u32::MAX;
    assert!(says(run(drive), "USB NOT READY"));

    // A read of USB.BIN refused once: the stalled data stage cleared, and the read tried again.
    let mut drive = drive_with(PARTITION, &usb_bin);
    (drive.refuse_reads, drive.refuse_after) = (1, 10);
    assert!(run(drive).is_none() && ram()[..usb_bin.len()] == usb_bin[..]);
    let d = &usb().drive;
    assert!(d.bot_resets == 0 && !d.halted_in);
    // Refused each of the three times it's tried.
    let mut drive = drive_with(PARTITION, &usb_bin);
    (drive.refuse_reads, drive.refuse_after) = (3, 10);
    assert!(says(run(drive), "USB READ ERROR"));
    // Refused from the first sector: the loader can't tell that from a drive that isn't FAT32.
    let mut drive = drive_with(PARTITION, &usb_bin);
    drive.refuse_reads = 3;
    assert!(says(run(drive), "USB NOT FAT32"));

    // Pulled out partway through USB.BIN.
    let mut drive = drive_with(PARTITION, &usb_bin);
    drive.pull_after_reads = 12;
    assert!(says(run(drive), "USB READ ERROR"));

    // No drive, not a drive, not 512-byte sectors.
    let mut drive = drive_with(PARTITION, &usb_bin);
    drive.attached = false;
    assert!(says(run(drive), "NO USB DRIVE"));
    let mut drive = drive_with(PARTITION, &usb_bin);
    drive.no_bulk = true;
    assert!(says(run(drive), "NOT A USB DRIVE"));
    let mut drive = drive_with(PARTITION, &usb_bin);
    drive.block = 2048;
    assert!(says(run(drive), "USB NOT 512 B"));

    // What's on it: not FAT32, no USB.BIN (only MENU.BIN), a damaged USB.BIN, one too big.
    let mut drive = drive_with(PARTITION, &usb_bin);
    drive.disk[(PARTITION * 512 + 17) as usize] = 2; // FAT12/16's root entry count
    assert!(says(run(drive), "USB NOT FAT32"));
    let mut image = Image::new(PARTITION);
    let mboot = image.folder(2, 0, b"MBOOT      ");
    image.file(mboot, 0, b"MENU    BIN", &stage2(600));
    assert!(says(run(Drive::new(image.disk)), "NO /MBOOT/USB.BIN"));
    let mut bad = usb_bin.clone();
    bad[100] ^= 1;
    assert!(says(run(drive_with(PARTITION, &bad)), "BAD USB.BIN"));
    let mut image = Image::new(PARTITION);
    let mboot = image.folder(2, 0, b"MBOOT      ");
    image.entry(mboot, 0, b"USB     BIN", 0x20, 2, STAGE2_MAX as u32 + 1);
    assert!(says(run(Drive::new(image.disk)), "BAD USB.BIN"));

    println!(
        "PASS: USB DRIVE: enumeration, the bulk-only transport and /MBOOT/USB.BIN on a simulated drive, and its \
         refusals: absent, not a drive, sector size, not ready, refused and pulled reads, missing and bad files"
    );
}
