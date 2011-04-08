
#include <avr/io.h>

// PB5: clock (pin 1)
// PB6: data  (pin 2)
//
// PB7: data  (pin 3)
// PB4: clock (pin 4)
void mouse_tx(uint8_t m, uint8_t b) {
   uint8_t i;
   uint8_t clock = (1 << 5);
   uint8_t data = (1 << 6);
   if( m == 1 ) {
      clock = (1 << 4);
      data = (1 << 7);
   }
   // transmit LSB-first
   // odd parity

   // clock low
   PORTB &= ~clock;
   DDRB |= clock;
   // wait 100 us
   delay_us(100);

   // data line low
   DDRB |= data;
   PORTB &= ~data;

   delay_us(10);

   // clock high
   PORTB |= clock;
   DDRB &= ~clock;

   uint8_t p = 1;
   for( i=0; i < 8; i++ ) {
      // wait for clock low
      while( PINB & clock );

      // set output data
      if( b&0x1 ) {
         PORTB |= data;
      } else {
         PORTB &= ~data;
      }

      // wait for clock high
      while( !(PINB & clock) );

      if( b & 0x1 ) p++;
      b >>= 1;
   }

   // wait for clock low
   while( PINB & clock );

   // set parity
   //  p will be even if odd number of bits set
   //  p will be off if even number of bits set
   if( p&0x1 ) {
      PORTB |= data;
   } else {
      PORTB &= ~data;
   }

   // wait for clock high
   while( !(PINB & clock) );

   // wait for clock low
   while( PINB & clock );

   // stop bit: release data line
   DDRB &= ~data;
   PORTB |= data;

   // wait for data low
   while( PINB & data );
   // wait for clock low
   while( PINB & clock );
   // wait for data and clock high
   while( !(PINB & data) || !(PINB & clock) );
}

// wait and receive a byte from the mouse
// PB5: clock (pin 1)
// PB6: data  (pin 2)
//
// PB7: data  (pin 3)
// PB4: clock (pin 4)
uint8_t mouse_rx(uint8_t m) {
   // clock data in on falling edge of clock
   uint8_t i;
   uint8_t b = 0;
   uint8_t p = 0;
   uint8_t clock = (1 << 5);
   uint8_t data = (1 << 6);

   if( m == 1 ) {
      clock = (1 << 4);
      data = (1 << 7);
   }

   // wait for clock low
   while( PINB & clock );

   // check stop bit
   if( PINB & data ) return 0;


   // receive data
   for( i=0; i<8; i++ ) {
      // wait for clock high
      while( !(PINB & clock) );
      // wait for clock low
      while( PINB & clock );
      b >>= 1;
      if( PINB & data ) {
         b |= 0x80;
         p++;
      }
   }

   // wait for clock high
   while( !(PINB & clock) );
   // wait for clock low
   while( PINB & clock );
   // receive parity bit
   if( PINB & data ) {
      p++;
   }
   if( !(p & 0x1) ) {
      b = 0;
   }

   // clock in stop bit. don't actually care about it
   // wait for clock high
   while( !(PINB & clock) );
   // wait for clock low
   while( PINB & clock );
   // wait for clock high
   while( !(PINB & clock) );
   return b;
}

void mouse_init() {
   mouse_tx(0, 0xF0);
   mouse_tx(1, 0xF0);
}

