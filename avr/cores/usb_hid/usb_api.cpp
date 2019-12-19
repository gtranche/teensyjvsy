/* USB API for Teensy USB Development Board
 * http://www.pjrc.com/teensy/teensyduino.html
 * Copyright (c) 2008 PJRC.COM, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include "usb_common.h"
#include "usb_private.h"
#include "usb_api.h"
#include "wiring.h"

void usb_keyboard_class::pressModifier(uint8_t code) {
	keyboard_report_data[0] |= code;
}

void usb_keyboard_class::releaseModifier(uint8_t code) {
	keyboard_report_data[0] &= ~(code);
}

void usb_keyboard_class::pressKey(uint8_t code) {
	if ((code >> 3) < KEYBOARD_SIZE - 2) {
		keyboard_report_data[(code >> 3) + 1] |= 1 << (code & 7);
	}
}

void usb_keyboard_class::releaseKey(uint8_t code) {
	if ((code >> 3) < KEYBOARD_SIZE - 2) {
		keyboard_report_data[(code >> 3) + 1] &= ~(1 << (code & 7));
	}
}

void usb_keyboard_class::releaseAll(void) {
	for (uint8_t i = 0; i < KEYBOARD_SIZE; i++) {
		keyboard_report_data[i] = 0;
	}
}

void usb_keyboard_class::send(void) {
	uint8_t intr_state, timeout, i;
	if (!usb_configuration)
		return;
	intr_state = SREG;
	cli();
	UENUM = KEYBOARD_ENDPOINT;
	timeout = UDFNUML + 50;
	while (1) {
		// are we ready to transmit?
		if (UEINTX & (1 << RWAL))
			break;
		SREG = intr_state;
		// has the USB gone offline?
		if (!usb_configuration)
			return;
		// have we waited too long?
		if (UDFNUML == timeout)
			return;
		// get ready to try checking again
		intr_state = SREG;
		cli();
		UENUM = KEYBOARD_ENDPOINT;
	}
	for (i = 0; i < KEYBOARD_SIZE; i++) {
		UEDATX = keyboard_report_data[i];
	}
	UEINTX = 0x3A;
	keyboard_idle_count = 0;
	SREG = intr_state;
}

void usb_joystick_class::send_now(void)
{
        uint8_t intr_state, timeout;

        if (!usb_configuration) return;
        intr_state = SREG;
        cli();
        UENUM = m_endpoint;
        timeout = UDFNUML + 50;
        while (1) {
                // are we ready to transmit?
                if (UEINTX & (1<<RWAL)) break;
                SREG = intr_state;
                // has the USB gone offline?
                if (!usb_configuration) return;
                // have we waited too long?
                if (UDFNUML == timeout) return;
                // get ready to try checking again
                intr_state = SREG;
                cli();
                UENUM = m_endpoint;
        }
        UEDATX = m_data[0];
        UEDATX = m_data[1];
        UEDATX = m_data[2];
        UEDATX = m_data[3];
        UEDATX = m_data[4];
        UEDATX = m_data[5];
        UEDATX = m_data[6];
        UEDATX = m_data[7];
        UEDATX = m_data[8];
        UEDATX = m_data[9];
        UEDATX = m_data[10];
        UEDATX = m_data[11];
        UEINTX = 0x3A;
        SREG = intr_state;
}




static volatile uint8_t prev_byte=0;

void usb_serial_class::begin(long speed)
{
	// make sure USB is initialized
	usb_init();
	uint16_t begin_wait = (uint16_t)millis();
	while (1) {
		if (usb_configuration) {
			delay(200);  // a little time for host to load a driver
			return;
		}
		if (usb_suspended) {
			uint16_t begin_suspend = (uint16_t)millis();
			while (usb_suspended) {
				// must remain suspended for a while, because
				// normal USB enumeration causes brief suspend
				// states, typically under 0.1 second
				if ((uint16_t)millis() - begin_suspend > 250) {
					return;
				}
			}
		}
		// ... or a timout (powered by a USB power adaptor that
		// wiggles the data lines to keep a USB device charging)
		if ((uint16_t)millis() - begin_wait > 2500) return;
	}
	prev_byte = 0;
}

void usb_serial_class::end()
{
	usb_shutdown();
	delay(25);
}



// number of bytes available in the receive buffer
int usb_serial_class::available()
{
        uint8_t c;

	c = prev_byte;  // assume 1 byte static volatile access is atomic
	if (c) return 1;
	c = readnext();
	if (c) {
		prev_byte = c;
		return 1;
	}
	return 0;
}

// get the next character, or -1 if nothing received
int usb_serial_class::read()
{
	uint8_t c;

	c = prev_byte;
	if (c) {
		prev_byte = 0;
		return c;
	}
	c = readnext();
	if (c) return c;
	return -1;
}

int usb_serial_class::peek()
{
	uint8_t c;
	
	c = prev_byte;
	if (c) return c;
	c = readnext();
	if (c) {
		prev_byte = c;
		return c;
	}
	return -1;
}

// get the next character, or 0 if nothing
uint8_t usb_serial_class::readnext(void)
{
        uint8_t c, intr_state;

        // interrupts are disabled so these functions can be
        // used from the main program or interrupt context,
        // even both in the same program!
        intr_state = SREG;
        cli();
        if (!usb_configuration) {
                SREG = intr_state;
                return 0;
        }
        UENUM = DEBUG_RX_ENDPOINT;
try_again:
        if (!(UEINTX & (1<<RWAL))) {
                // no packet in buffer
                SREG = intr_state;
                return 0;
        }
        // take one byte out of the buffer
        c = UEDATX;
	if (c == 0) {
		// if we see a zero, discard it and
		// discard the rest of this packet
		UEINTX = 0x6B;
		goto try_again;
	}
        // if this drained the buffer, release it
        if (!(UEINTX & (1<<RWAL))) UEINTX = 0x6B;
        SREG = intr_state;
        return c;
}

// discard any buffered input
void usb_serial_class::flush()
{
        uint8_t intr_state;

        if (usb_configuration) {
                intr_state = SREG;
                cli();
		UENUM = DEBUG_RX_ENDPOINT;
                while ((UEINTX & (1<<RWAL))) {
                        UEINTX = 0x6B;
                }
                SREG = intr_state;
        }
	prev_byte = 0;
}

// transmit a character.
size_t usb_serial_class::write(uint8_t c)
{
        //static uint8_t previous_timeout=0;
        uint8_t timeout, intr_state;

        // if we're not online (enumerated and configured), error
        if (!usb_configuration) goto error;
        // interrupts are disabled so these functions can be
        // used from the main program or interrupt context,
        // even both in the same program!
        intr_state = SREG;
        cli();
        UENUM = DEBUG_TX_ENDPOINT;
        // if we gave up due to timeout before, don't wait again
#if 0
	// this seems to be causig a lockup... why????
        if (previous_timeout) {
                if (!(UEINTX & (1<<RWAL))) {
                        SREG = intr_state;
                        return;
                }
                previous_timeout = 0;
        }
#endif
        // wait for the FIFO to be ready to accept data
        timeout = UDFNUML + TRANSMIT_TIMEOUT;
        while (1) {
                // are we ready to transmit?
                if (UEINTX & (1<<RWAL)) break;
                SREG = intr_state;
                // have we waited too long?  This happens if the user
                // is not running an application that is listening
                if (UDFNUML == timeout) {
                        //previous_timeout = 1;
			goto error;
		}
                // has the USB gone offline?
                if (!usb_configuration) goto error;
                // get ready to try checking again
                intr_state = SREG;
                cli();
                UENUM = DEBUG_TX_ENDPOINT;
        }
        // actually write the byte into the FIFO
        UEDATX = c;
        // if this completed a packet, transmit it now!
        if (!(UEINTX & (1<<RWAL))) {
		UEINTX = 0x3A;
        	debug_flush_timer = 0;
	} else {
        	debug_flush_timer = TRANSMIT_FLUSH_TIMEOUT;
	}
        SREG = intr_state;
	return 1;
error:
	setWriteError();
	return 0;
}


// These are Teensy-specific extensions to the Serial object

// immediately transmit any buffered output.
// This doesn't actually transmit the data - that is impossible!
// USB devices only transmit when the host allows, so the best
// we can do is release the FIFO buffer for when the host wants it
void usb_serial_class::send_now(void)
{
        uint8_t intr_state;

        intr_state = SREG;
        cli();
        if (debug_flush_timer) {
                UENUM = DEBUG_TX_ENDPOINT;
		while ((UEINTX & (1<<RWAL))) {
			UEDATX = 0;
		}
                UEINTX = 0x3A;
                debug_flush_timer = 0;
        }
        SREG = intr_state;
}

uint32_t usb_serial_class::baud(void)
{
	return ((uint32_t)DEBUG_TX_SIZE * 10000 / DEBUG_TX_INTERVAL);
}

uint8_t usb_serial_class::stopbits(void)
{
	return 1;
}

uint8_t usb_serial_class::paritytype(void)
{
	return 0;
}

uint8_t usb_serial_class::numbits(void)
{
	return 8;
}

uint8_t usb_serial_class::dtr(void)
{
	return 1;
}

uint8_t usb_serial_class::rts(void)
{
	return 1;
}

usb_serial_class::operator bool()
{
	if (usb_configuration) return true;
	return false;
}


// Preinstantiate Objects //////////////////////////////////////////////////////

usb_serial_class	Serial = usb_serial_class();
usb_keyboard_class	Keyboard = usb_keyboard_class();
usb_joystick_class	Joystick = usb_joystick_class(JOYSTICK_INTERFACE, JOYSTICK_ENDPOINT, JOYSTICK_SIZE, joystick_report_data);
usb_joystick_class	Joystick2 = usb_joystick_class(JOYSTICK2_INTERFACE, JOYSTICK2_ENDPOINT, JOYSTICK2_SIZE, joystick2_report_data);

