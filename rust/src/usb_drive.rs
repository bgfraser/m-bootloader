// SPDX-License-Identifier: MIT
//! The FM-1's USB DRIVE item: a USB flash drive on USB0, with the unit as the host, so that /MBOOT/USB.BIN loads from
//! it as /MBOOT/MENU.BIN does from a card (sdboot.rs). Full speed, polled, read-only. The drive needs 5 V from
//! elsewhere, such as a Y-cable with a power injector, as the unit doesn't supply VBUS.
//!
//! After M-Unbricker's host driver and bulk-only transport (m-unbricker/src/husb.c, bot.c), which have run on an FM-1
//! with the ROM's download-mode disk, and m-apps' apps/usbflash/stick.c, cut to what reading a drive needs: control
//! transfers with an IN data stage or none, one bulk IN and one bulk OUT pipe, and TEST UNIT READY, REQUEST SENSE,
//! READ CAPACITY (10) and READ (10). The controller's registers go through the MIDI driver's spaced accesses.
use crate::hw::{Reg, Text, bit, dma_address, sync_cpu};
use crate::usb_midi::{memory_sync, sie_read, sie_write};
use crate::{Aligned, Global, text};

const CON0: Reg = Reg::at::<0x11800>();
const EP0_CNT: Reg = CON0.add(8);
const EP1_CNT: Reg = CON0.add(0xc);
const EP0_ADR: Reg = CON0.add(0x18);
const EP1_TADR: Reg = CON0.add(0x1c);
const EP1_RADR: Reg = CON0.add(0x20);
const IO_CON0: Reg = Reg::at::<0x51000>();
const IO_CON1: Reg = IO_CON0.add(4);
const CLK_CON1: Reg = Reg::at::<0x10010>();
/// Timer 0, free-running at the LSB clock, as M-Unbricker keeps time. hal_delay_us() sets it up again for itself.
const TIMER: Reg = Reg::at::<0x10400>();
const TIMER_COUNT: Reg = TIMER.add(4);
const TIMER_PERIOD: Reg = TIMER.add(8);
// The controller's own registers, reached through sie_read() and sie_write().
const FADDR: u32 = 0;
const POWER: u32 = 1;
const INTRUSBE: u32 = 11;
const INDEX: u32 = 14;
const DEVCTL: u32 = 15;
const TXMAXP: u32 = 16;
const CSR0: u32 = 17;
const TXCSR1: u32 = 17;
const TXCSR2: u32 = 18;
const RXMAXP: u32 = 19;
const RXCSR1: u32 = 20;
const RXCSR2: u32 = 21;
const COUNT0: u32 = 22;
const RXCOUNT1: u32 = 22;
const RXCOUNT2: u32 = 23;
const TXTYPE: u32 = 24;
const RXTYPE: u32 = 26;
/// The drive's address once it has one.
const ADDRESS: u32 = 1;
const TIMEOUT_MS: u32 = 5000;
pub type Block = [u8; 512];

#[derive(Clone, Copy, PartialEq)]
enum Fail {
    Stall,
    Error,
}

type Res<T> = Result<T, Fail>;

// What the controller's DMA reads and writes (a packet, and 4 bytes more for the DMA's overrun on receive), and DATA
// for what a control transfer or a short command brings back.
static EP0: Global<Aligned<72>> = Global::new(Aligned([0; 72]));
static TX: Global<Aligned<64>> = Global::new(Aligned([0; 64]));
static RX: Global<Aligned<72>> = Global::new(Aligned([0; 72]));
static DATA: Global<Aligned<256>> = Global::new(Aligned([0; 256]));

#[derive(Clone, Copy)]
struct State {
    ticks_per_ms: u32,
    mps0: u32,
    ep_in: u32,
    ep_out: u32,
    mps: u32,
    iface: u32,
    tag: u32,
    sectors: u32,
}

static STATE: Global<State> = Global::new(State {
    ticks_per_ms: 0,
    mps0: 0,
    ep_in: 0,
    ep_out: 0,
    mps: 0,
    iface: 0,
    tag: 0,
    sectors: 0,
});

/// Safety, for every use: nothing here is entered from what it calls, and each of these is borrowed by one function
/// at a time.
fn buffer<const N: usize>(b: &'static Global<Aligned<N>>) -> &'static mut [u8; N] {
    unsafe { &mut b.get().0 }
}

fn state() -> &'static mut State {
    unsafe { STATE.get() }
}

fn now() -> u32 {
    TIMER_COUNT.read()
}

fn ms_since(t0: u32) -> u32 {
    now().wrapping_sub(t0) / state().ticks_per_ms
}

fn wait_ms(ms: u32) {
    let t0 = now();
    while ms_since(t0) < ms {}
}

/// The controller as a host: PHY on, host pull-downs, endpoint 0's buffer.
fn start() {
    CON0.write(0);
    CON0.add(4).write(0);
    IO_CON0.write(0x0c);
    wait_ms(20);
    // USB clock: PLL 48 MHz
    CLK_CON1.clear(3);
    IO_CON0.write(0);
    IO_CON1.write(0);
    wait_ms(2);
    IO_CON0.write(bit(9) | bit(10) | bit(4) | bit(5));
    // PHY on, SIE out of reset, VBUS valid; ID 0 = host
    CON0.set(0x25);
    wait_ms(2);
    EP0_ADR.write(dma_address(buffer(&EP0).as_mut_ptr()));
}

/// A session with the device: true once it has connected at full speed (or low, not expected).
fn session() -> bool {
    sie_write(POWER, 0x60);
    for r in 7..=INTRUSBE {
        sie_write(r, 0);
    }
    sie_write(DEVCTL, 3);
    let t0 = now();
    while ms_since(t0) < 500 {
        let d = sie_read(DEVCTL);
        if d & 4 != 0 && d & 0x60 != 0 {
            if d & 0x40 == 0 {
                CON0.set(2);
            }
            return true;
        }
        wait_ms(10);
    }
    false
}

/// Waits up to `timeout` ms for D+ to be pulled up for 100 ms, then starts a session.
fn wait_attach(timeout: u32) -> bool {
    let (t0, mut high) = (now(), 0);
    while ms_since(t0) < timeout {
        if CON0.read() & bit(17) != 0 {
            high += 1;
            if high >= 100 {
                return session();
            }
        } else {
            high = 0;
        }
        wait_ms(1);
    }
    false
}

fn attached() -> bool {
    (0..16).any(|_| CON0.read() & (bit(17) | bit(16)) != 0)
}

fn bus_reset() {
    for _ in 0..10 {
        let p = sie_read(POWER);
        sie_write(POWER, p & 0xf7 | 8);
        if p & 8 != 0 {
            break;
        }
    }
    wait_ms(50);
    sie_write(POWER, sie_read(POWER) & 0xf7);
    // 40 ms of recovery, and the 20 ms M-Unbricker waits after each reset
    wait_ms(60);
}

fn csr0() -> u32 {
    sie_write(INDEX, 0);
    sie_read(CSR0)
}

fn csr0_write(v: u32) {
    sie_write(INDEX, 0);
    sie_write(CSR0, v);
}

/// Until endpoint 0's status has `want` in `mask`; a stall, an error or a second without it ends it.
fn ep0_wait(mask: u32, want: u32) -> Res<()> {
    let t0 = now();
    loop {
        let c = csr0();
        let fail = if c & 4 != 0 {
            Fail::Stall
        } else if c & 0x10 != 0 || ms_since(t0) > 1000 {
            Fail::Error
        } else if c & mask == want {
            return Ok(());
        } else {
            continue;
        };
        csr0_write(0);
        return Err(fail);
    }
}

/// A control transfer with no data stage, or an IN one into `data`: the bytes it brought.
fn control(address: u32, setup: [u8; 8]) -> Res<usize> {
    let (ep0, data) = (buffer(&EP0), buffer(&DATA));
    let mps0 = state().mps0 as usize;
    let mut left = u16::from_le_bytes([setup[6], setup[7]]) as usize;
    let mut got = 0;
    sie_write(FADDR, address);
    ep0[..8].copy_from_slice(&setup);
    sync_cpu();
    EP0_CNT.write(8);
    // SETUP
    csr0_write(0x0a);
    ep0_wait(2, 0)?;
    if left == 0 {
        // status IN
        csr0_write(0x60);
        let r = ep0_wait(1, 1);
        csr0_write(0);
        return r.map(|_| 0);
    }
    csr0_write(0x20);
    while left != 0 {
        ep0_wait(1, 1)?;
        sie_write(INDEX, 0);
        let n = (sie_read(COUNT0) as usize).min(left).min(mps0);
        memory_sync();
        data[got..got + n].copy_from_slice(&ep0[..n]);
        got += n;
        left = if n < mps0 { 0 } else { left - n };
        csr0_write(if left != 0 { 0x20 } else { 0 });
    }
    // status OUT: zero-length
    EP0_CNT.write(0);
    csr0_write(0x42);
    ep0_wait(2, 0).map(|_| got)
}

/// A standard or class request, sent again twice if it fails other than by a stall.
fn request(address: u32, kind: u8, req: u8, value: u32, index: u32, len: u32) -> Res<usize> {
    let [v0, v1, ..] = value.to_le_bytes();
    let [i0, i1, ..] = index.to_le_bytes();
    let [l0, l1, ..] = len.to_le_bytes();
    let mut r = Err(Fail::Error);
    for _ in 0..3 {
        r = control(address, [kind, req, v0, v1, i0, i1, l0, l1]);
        if r != Err(Fail::Error) {
            break;
        }
        wait_ms(20);
    }
    r
}

/// Endpoint 1's pipes, both toggles at DATA0.
fn bulk_setup() {
    EP1_RADR.write(dma_address(buffer(&RX).as_mut_ptr()));
    EP1_TADR.write(dma_address(buffer(&TX).as_mut_ptr()));
    sie_write(INDEX, 1);
    sie_write(RXMAXP, 0xff);
    sie_write(RXCSR2, 0);
    sie_write(TXMAXP, 0xff);
    sie_write(TXCSR2, 0);
    reset_toggle(true);
    reset_toggle(false);
}

fn reset_toggle(input: bool) {
    sie_write(INDEX, 1);
    if input {
        sie_write(RXCSR1, 0x90);
    } else {
        sie_write(TXCSR1, 0x48);
    }
}

/// One packet of `n` bytes from the transmit buffer: a command block.
fn bulk_out(n: u32) -> Res<()> {
    let (t0, mut errors) = (now(), 0);
    sie_write(FADDR, ADDRESS);
    sync_cpu();
    'send: loop {
        sie_write(INDEX, 1);
        sie_write(TXTYPE, 0x20 | state().ep_out);
        EP1_CNT.write(n);
        sync_cpu();
        sie_write(TXCSR1, 1);
        loop {
            let c = sie_read(TXCSR1);
            if c & 0x20 != 0 {
                sie_write(TXCSR1, c & !0x21);
                return Err(Fail::Stall);
            }
            // NAK timeout or three errors: send it again
            if c & 0x84 != 0 {
                sie_write(TXCSR1, c & !0x85);
                errors += 1;
                if errors > 5 {
                    return Err(Fail::Error);
                }
                continue 'send;
            }
            if c & 1 == 0 {
                return Ok(());
            }
            if ms_since(t0) > TIMEOUT_MS {
                sie_write(TXCSR1, 8);
                return Err(Fail::Error);
            }
        }
    }
}

fn ask_packet() {
    sie_write(RXTYPE, 0x20 | state().ep_in);
    sie_write(RXCSR1, 0x20);
}

/// Up to `to.len()` bytes, ending early at a short packet: the bytes received.
fn bulk_in(to: &mut [u8]) -> Res<usize> {
    let rx = buffer(&RX);
    let mps = state().mps as usize;
    let (mut t0, mut errors, mut got) = (now(), 0, 0);
    sie_write(FADDR, ADDRESS);
    sie_write(INDEX, 1);
    ask_packet();
    loop {
        let c = sie_read(RXCSR1);
        if c & 1 != 0 {
            let n = (sie_read(RXCOUNT1) | sie_read(RXCOUNT2) << 8) as usize;
            memory_sync();
            let take = n.min(to.len() - got).min(mps);
            to[got..got + take].copy_from_slice(&rx[..take]);
            // release the buffer
            sie_write(RXCSR1, 0x10);
            got += take;
            if got == to.len() || n < mps {
                return Ok(got);
            }
            ask_packet();
            t0 = now();
        } else if c & 0x40 != 0 {
            sie_write(RXCSR1, 0);
            return Err(Fail::Stall);
        } else if c & 0x0c != 0 {
            // an error or a NAK timeout: ask again
            errors += 1;
            if errors > 5 {
                sie_write(RXCSR1, 0);
                return Err(Fail::Error);
            }
            sie_write(RXCSR1, 0x20);
        } else if ms_since(t0) > TIMEOUT_MS {
            sie_write(RXCSR1, 0);
            sie_write(RXCSR1, 0x10);
            return Err(Fail::Error);
        }
    }
}

/// The first interface with a bulk IN and a bulk OUT endpoint, from a configuration descriptor.
fn find_bulk(mut p: &[u8]) -> bool {
    let s = state();
    let (mut input, mut output) = (0, 0);
    while let [len, kind, ..] = *p {
        let len = len as usize;
        if len < 2 || len > p.len() {
            break;
        }
        let d = &p[..len];
        if kind == 4 && len >= 3 {
            s.iface = d[2] as u32;
            (input, output) = (0, 0);
        } else if kind == 5 && len >= 6 && d[3] & 3 == 2 {
            if d[2] & 0x80 != 0 {
                input = d[2] as u32 & 15;
                s.mps = match u16::from_le_bytes([d[4], d[5]]) {
                    m @ 1..=64 => m as u32,
                    _ => 64,
                };
            } else {
                output = d[2] as u32 & 15;
            }
            if input != 0 && output != 0 {
                (s.ep_in, s.ep_out) = (input, output);
                return true;
            }
        }
        p = &p[len..];
    }
    false
}

/// Address, configuration and bulk endpoints. What to show if it fails.
fn enumerate() -> Option<Text> {
    const ERROR: Text = text!("USB ENUM ERROR");
    let s = state();
    s.mps0 = 8;
    bus_reset();
    // the first 8 bytes of the device descriptor at address 0: endpoint 0's size
    if request(0, 0x80, 6, 0x100, 0, 8) != Ok(8) {
        return Some(ERROR);
    }
    s.mps0 = match buffer(&DATA)[7] {
        m @ 1..=64 => m as u32,
        _ => 8,
    };
    bus_reset();
    // SET_ADDRESS
    if request(0, 0, 5, ADDRESS, 0, 0).is_err() {
        return Some(ERROR);
    }
    wait_ms(10);
    // the whole configuration descriptor, as far as 255 bytes (a drive's is about 32)
    let Ok(n) = request(ADDRESS, 0x80, 6, 0x200, 0, 255) else {
        return Some(ERROR);
    };
    let data = buffer(&DATA);
    if n < 9 || !find_bulk(&data[..n]) {
        return Some(text!("NOT A USB DRIVE"));
    }
    // SET_CONFIGURATION
    if request(ADDRESS, 0, 9, data[5] as u32, 0, 0).is_err() {
        return Some(ERROR);
    }
    bulk_setup();
    None
}

fn clear_halt(input: bool) {
    let s = state();
    let ep = if input { 0x80 | s.ep_in } else { s.ep_out };
    let _ = request(ADDRESS, 2, 1, 0, ep, 0);
    reset_toggle(input);
}

#[derive(PartialEq)]
enum Status {
    Passed,
    Refused,
    Failed,
}

/// One SCSI command through the bulk-only transport, with `to` for what it reads (not DATA): the status, and the
/// bytes read.
fn command(cdb: &[u8], to: &mut [u8]) -> (Status, usize) {
    let s = state();
    s.tag = s.tag.wrapping_add(1);
    let tag = s.tag.to_le_bytes();
    let cbw = &mut buffer(&TX)[..31];
    cbw.fill(0);
    cbw[..4].copy_from_slice(b"USBC");
    cbw[4..8].copy_from_slice(&tag);
    cbw[8..12].copy_from_slice(&(to.len() as u32).to_le_bytes());
    cbw[12] = 0x80;
    cbw[14] = cdb.len() as u8;
    cbw[15..15 + cdb.len()].copy_from_slice(cdb);
    let mut moved = 0;
    let mut status = Status::Failed;
    if bulk_out(31).is_ok() {
        let data = match if to.is_empty() { Ok(0) } else { bulk_in(to) } {
            Ok(n) => {
                moved = n;
                Ok(())
            }
            // a stalled data stage: the status follows
            Err(Fail::Stall) => {
                clear_halt(true);
                Ok(())
            }
            Err(e) => Err(e),
        };
        if data.is_ok() {
            let csw = &mut buffer(&DATA)[..13];
            let mut r = bulk_in(csw);
            if r == Err(Fail::Stall) {
                clear_halt(true);
                r = bulk_in(csw);
            }
            if r == Ok(13) && csw[..4] == *b"USBS" && csw[4..8] == tag {
                status = match csw[12] {
                    0 => Status::Passed,
                    1 => Status::Refused,
                    _ => Status::Failed,
                };
            }
        }
    }
    if status == Status::Failed && attached() {
        // reset recovery: bulk-only mass storage reset, then both endpoints' halts cleared
        let _ = request(ADDRESS, 0x21, 0xff, 0, s.iface, 0);
        clear_halt(true);
        clear_halt(false);
    }
    (status, moved)
}

/// TEST UNIT READY until the drive says so, asking what's wrong after each refusal (some drives want that).
fn ready() -> bool {
    let t0 = now();
    while ms_since(t0) < 10_000 {
        match command(&[0; 6], &mut []).0 {
            Status::Passed => return true,
            Status::Refused => {
                let mut sense = [0; 18];
                command(&[3, 0, 0, 0, 18, 0], &mut sense);
            }
            Status::Failed if !attached() => return false,
            Status::Failed => {}
        }
        wait_ms(100);
    }
    false
}

/// Starts the controller as a host, waits up to `wait_ms` for a drive and makes it ready. None, or what to show.
pub fn open(lsb_mhz: u32, wait_ms: u32) -> Option<Text> {
    TIMER.write(bit(14));
    TIMER_COUNT.write(0);
    TIMER_PERIOD.write(!0);
    TIMER.write(1);
    let s = state();
    s.ticks_per_ms = lsb_mhz * 1000;
    s.sectors = 0;
    start();
    if !wait_attach(wait_ms) {
        return Some(text!("NO USB DRIVE"));
    }
    if let Some(error) = enumerate() {
        return Some(error);
    }
    let not_ready = Some(text!("USB NOT READY"));
    let mut c = [0; 8];
    for _ in 0..4 {
        if !ready() {
            break;
        }
        // READ CAPACITY (10): the last sector, and the sector's size
        if command(&[0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0], &mut c) == (Status::Passed, 8) {
            if c[4..] != [0, 0, 2, 0] {
                return Some(text!("USB NOT 512 B"));
            }
            s.sectors = u32::from_be_bytes([c[0], c[1], c[2], c[3]]).wrapping_add(1);
            return None;
        }
    }
    not_ready
}

/// One 512-byte sector, tried three times.
pub fn read(lba: u32, to: &mut Block) -> bool {
    let [a, b, c, d] = lba.to_be_bytes();
    for _ in 0..3 {
        if lba >= state().sectors || !attached() {
            return false;
        }
        if command(&[0x28, 0, a, b, c, d, 0, 0, 1, 0], to) == (Status::Passed, 512) {
            return true;
        }
    }
    false
}

/// Takes the controller down, so that what runs next starts USB from scratch.
pub fn close() {
    CON0.write(0);
    IO_CON0.write(0);
}
