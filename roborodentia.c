/* map.c
 * a mapping program for my robot
 * NOTES:
 * sizeof(int) = 2;
 */
#include "polybot_library/globals.h"
#include "system.h"
#include "serial.h"

#include <math.h>

#include <avr/io.h>

struct pos {
   int16_t x;
   int16_t y;
};

struct pos mouse_read(uint8_t m) {
   uint8_t i;
   uint8_t b[4];
   int16_t x;
   int16_t y;
   struct pos move;
   mouse_tx(m, 0xEB);
   for( i=0; i<4; i++ ) {
      b[i] = mouse_rx(m);
   }
   x = b[2];
   y = b[3];
   if( b[1] & 0x10 ) {
      x |= 0xFF00;
   }
   if( b[1] & 0x20 ) {
      y |= 0xFF00;
   }

   move.x = x;
   move.y = y;

   return move;
}

int main(void)
{
   struct pos m1;
   struct pos m2;

   // position
   double x = 0;
   double y = 0;
   double theta = 0;

   double d = 1065;

   // temporary variables
   int16_t dy;
   //int16_t x1;
   //int16_t x2;
   double alpha;
   double theta2;
   double r1;
   double r2;
   double cx;
   double cy;

   DDRB &= ~(0xF0); // PB4-PB7 as input
   PORTB |= 0xF0;   // PB4-PB7 pull up

   initialize();
   //servo_init();
   //serial_init();

   //system_init();
   //schedule(0);
   //priority(100);
   mouse_init();

   clear_screen();
   print_string("Hello Mouse");

   while(1) {
      m1 = mouse_read(0);
      m2 = mouse_read(1);
      //m1.x = ((int16_t)knob()) - 128;

      clear_screen();
      print_string("(");
      print_int(m1.x);
      print_string(",");
      print_int(m1.y);
      print_string(") (");
      print_int(m2.x);
      print_string(",");
      print_int(m2.y);
      print_string(")");

      next_line();

      dy = (m1.y + m2.y) / 2;
      //x1 = m1.x;
      //x2 = m2.x;

      if( m1.x != m2.x ) {
         alpha = (d * m2.x) / (m1.x - m2.x);
         theta2 = theta - ((m1.x - m2.x) / d);
         r1 = alpha - (dy/2) + (d/2);
         r2 = r1 + dy;
         cx = x - (r1 * cos(theta));
         cy = y - (r1 * sin(theta));
         x = cx + (r2 * cos(theta2));
         y = cy + (r2 * sin(theta2));
         theta = theta2;

      } else {
         x += (dy * cos(theta)) + (m1.x * sin(theta));
         y += (dy * sin(theta)) - (m1.x * cos(theta));
      }

      while( theta > M_PI ) theta -= 2*M_PI;
      while( theta < -M_PI ) theta += 2*M_PI;

      //x += (double)x1;

      //y += (double)dy;

      //print_string("(");
      print_fp(x);
      print_string(" ");
      print_fp(y);
      print_string(" ");
      print_fp(theta);
      //print_string(")");

      //delay_ms(250);
   }

}
