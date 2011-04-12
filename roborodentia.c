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

#define TARGET_DIST 100

inline double max(double a, double b) {
   return a>b?a:b;
}

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

struct T {
   double x;
   double y;
};

struct T targets[] = {
   0.0,0.0,
   2000.0,0.0,
   2000.0,2000.0,
   0.0,2000.0,
};

int main(void)
{
   struct pos m1;
   struct pos m2;

   // current target
   uint8_t target = 1;

   // position
   double x = 0;
   double y = 0;
   double theta = M_PI / 2;

   static double d = 870;

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

   double vlf = 0.0; 
   double vrf = 0.0; 
   double vlr = 0.0; 
   double vrr = 0.0; 

   double vlf_old = 0.0; 
   double vrf_old = 0.0; 
   double vlr_old = 0.0; 
   double vrr_old = 0.0; 

   DDRB &= ~(0xF0); // PB4-PB7 as input
   PORTB |= 0xF0;   // PB4-PB7 pull up

   initialize();
   print_string("Remove programming");
   next_line();
   print_string("cable");
   //servo_init();
   //serial_init();
   motor_init();

   //system_init();
   //schedule(0);
   //priority(100);
   mouse_init();

   clear_screen();
   print_string("Hello Mouse");

   while(!get_sw1());

   while(1) {
      tbi(PORTA, 4); // toggle LED
      m1 = mouse_read(0);
      m2 = mouse_read(1);
      m1.x = -m1.x;
      m1.y = -m1.y;
      //m1.x = ((int16_t)knob()) - 128;

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

      // target theta = 0; ignore
      cy = targets[target].y - y;
      cx = targets[target].x - x;
      theta2 = (M_PI/2) - theta;
      //theta2 = -(M_PI/2) - theta;

      // normalize theta2
      while( theta2 > M_PI ) theta2 -= M_PI;
      while( theta2 < -M_PI) theta2 += M_PI;


      if( hypot(cx, cy) < TARGET_DIST ) {
         target++;
         target %= 4;
         cy = targets[target].y - y;
         cx = targets[target].x - x;
      }

      // X points right; Y points forward
      double forward = cy*sin(theta) + cx*cos(theta);
      double right = cx*sin(theta) - cy*cos(theta);
      //double forward = 0.0;
      //double right = 0.0;
      double spin = theta2;
      //double spin = 0;

      // direction gains
      static double fg = 0.1; // forward gain
      static double sg = 0.4; // strafe gain
      static double rg = 80.0; // rotate gain

      if( forward > 250.0 ) forward = 250.0;
      if( forward < -250.0 ) forward = -250.0;

      if( right > 250.0 ) right = 250.0;
      if( right < -250.0 ) right = -250.0;

      // wheel speeds
      double vlf = fg*forward - sg*right - rg*spin;
      double vrf = fg*forward + sg*right - rg*spin;
      double vlr = fg*forward + sg*right + rg*spin;
      double vrr = fg*forward - sg*right + rg*spin;

      /*
      double vlf = fg*forward;
      double vrf = fg*forward;
      double vlr = fg*forward;
      double vrr = fg*forward;
      */

      double m = max(max(fabs(vlf), fabs(vrf)), max(fabs(vlr), fabs(vrr)));
      static double motor_max = 100.0;

      if( m > motor_max ) {
         double scale = m / motor_max;
         vlf /= scale;
         vrf /= scale;
         vlr /= scale;
         vrr /= scale;
      }

      static double max_delta = 5.0;
      double d = vlf - vlf_old;
      if( max_delta < d )
         vlf = vlf_old + max_delta;
      if( -max_delta > d )
         vlf = vlf_old - max_delta;

      d = vrf - vrf_old;
      if( max_delta < d )
         vrf = vrf_old + max_delta;
      if( -max_delta > d )
         vrf = vrf_old - max_delta;

      d = vlr - vlr_old;
      if( max_delta < d )
         vlr = vlr_old + max_delta;
      if( -max_delta > d )
         vlr = vlr_old - max_delta;

      d = vrr - vrr_old;
      if( max_delta < d )
         vrr = vrr_old + max_delta;
      if( -max_delta > d )
         vrr = vrr_old - max_delta;


      set_motor_power(0, (signed char)vlf);
      set_motor_power(1, (signed char)vrf);
      set_motor_power(2, (signed char)vlr);
      set_motor_power(3, (signed char)vrr);

      vlf_old = vlf;
      vrf_old = vrf;
      vlr_old = vlr;
      vrr_old = vrr;

      clear_screen();
      print_fp(x);
      print_string(" ");
      print_fp(y);

      next_line();

      print_int(target);
      print_string(" ");
      print_fp(theta);
      /*
      print_fp(cx);
      print_string(" ");
      print_fp(cy);

      next_line();

      print_int(target);
      print_string(" ");
      print_fp(theta2);
      */

      delay_ms(50);
   }

}
