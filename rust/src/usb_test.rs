// SPDX-License-Identifier: MIT
//! The FM-1's USB DRIVE item, tried from RAM before any bootloader with it is installed: M-Flasher's run-ram starts
//! this from download mode, and it writes nothing to flash.
//!
//! It sets the unit up as the RAM stage does from run-ram (stage.rs), then does what choosing the item does
//! (`usb_stage2`): waits for a drive, loads /MBOOT/USB.BIN from it and runs it. If that returns or fails, it says so
//! on the screen and resets. It's linked at 0x01C40000, out of the way of USB.BIN (0x01C28000, with its data above)
//! and of the item's USB buffers (0x01C16800).
//!
//! ```sh
//! m-flasher run-ram build/fm1/usb-test.bin --addr 0x01C40000
//! ```
use crate::display::{self, GREEN, RED, WHITE, YELLOW};
use crate::hw::irq_off;
use crate::platform::{flash_init, hal_clock_b, hal_delay_us, hal_early, hal_reset};
use crate::sdboot::usb_stage2;
use crate::text;

/// The LSB clock that hal_clock_b() gives.
const LSB_MHZ: u32 = 48;
/// How long the result stays on the screen before the reset.
const SHOW_S: u32 = 30;

/// From the entry (src/stage_start.S), as the ROM called it.
#[unsafe(no_mangle)]
pub extern "C" fn stage_main(_context: *mut u8, _stream_end: u32) -> ! {
    unsafe extern "C" {
        static mut __bss_start: u8;
        static __bss_end: u8;
    }
    irq_off();
    // Safety: this image's own data, which nothing has used yet.
    unsafe {
        core::ptr::write_bytes(
            &raw mut __bss_start,
            0,
            crate::hw::distance(&raw const __bss_start, &raw const __bss_end),
        )
    };
    hal_early();
    hal_clock_b();
    flash_init(1, 1);
    let screen = display::init(1);
    if screen {
        display::text(1, GREEN, text!("USB DRIVE TEST"));
        display::text(3, YELLOW, text!("LOOKING FOR DRIVE"));
    }
    let reason = usb_stage2(LSB_MHZ);
    // USB.BIN may have left the flash interface and the panel in any state.
    flash_init(1, 1);
    if display::init(1) {
        display::text(1, GREEN, text!("USB DRIVE TEST"));
        match reason {
            Some(why) => display::text(3, RED, why),
            None => display::text(3, WHITE, text!("USB.BIN RETURNED")),
        }
        display::text(5, WHITE, text!("RESTARTING IN 30 S"));
    }
    for _ in 0..SHOW_S {
        hal_delay_us(1_000_000);
    }
    hal_reset();
}
