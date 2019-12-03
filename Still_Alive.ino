/*
  ================================================================================
  TellyMate Demonstration.
  Portal ending credits song; "Still Alive"
  Demonstration of vector -> text rendering.
  Code by Nigel Batten.
  Contactable on <firstname>@batsocks.co.uk
  ================================================================================
  This demonstration is intended for running on a Batsocks TellyMate Shield,
  however, it will output a much simpler (albeit wider) display by commenting out
  the #define USE_TELLYMATE line
  This works well with the Serial Monitor provided with the Arduino IDE, but
  should work with almost any serial device. If you happen to have a serial
  enabled daisywheel/dot-matrix/teletype printer; go for it - send me a video!
  Arduino Required:
  ================
  Suitable for 328 based Arduinos (the majority).
  Untested on 168 based Arduinos (possibly not enough RAM)
  Not suitable for Mega8 based Arduinos.
  Extra Hardware required:
  =======================
  Piezo / 8-ohm speaker connected to pin 8.
  It'll run happily without one connected, but you will miss out on the whole
  son-et-lumiere experience.
  See the Arduino tutorial for tone() for more details on connecting a piezo or
  8-ohm speaker : http://arduino.cc/en/Tutorial/Tone
  ================================================================================
  The original scanline conversion routine was found here:
  http://code-heaven.blogspot.com/2009/10/simple-c-program-for-scan-line-polygon.html
  Although I've modified it quite heavily (e.g. horribly) to handle stroked polys.
  ================================================================================
*/
#include <avr/pgmspace.h>

#define BAUD_RATE 57600


// To Run this demo as a simple Serial demo, comment out the following line:
//#define USE_TELLYMATE

#ifdef USE_TELLYMATE
#define XSCALE 1.2
#define YOFFSET -16
#define SCANLINE_WIDTH 38
#else
#define XSCALE 1.9
#define YOFFSET 8
#define SCANLINE_WIDTH 60
#define SIMPLE
#endif


#define CHAR_ESC "\x1B" // character 27 is <ESC>
#define CHAR_DLE '\x10' // data link escape
#define CHAR_SPACE ' '
#define GRAPHICS_FONTBANK 0


unsigned long m_nextevent = 0 ; // time of the next event
byte *m_P_event = 0 ; // pointer to next event

// the number of milliseconds per 'beat' (e.g. the shortest note)
// in this case, the shortest note is a semiquaver.
// With an andante tempo (let's say 100bpm), this
// equates to 0.6s per crochet, or 0.15s per semiquaver.
byte m_millis_per_beat = 130 ;


/*
  =============================================================
  Vector based pictures
  These are a very simple format;
  Each 'drawing' consists of a single byte containing the number
  of polygons, followed by the polygon data.
  Each polygon consists of 1 byte holding the number vertices in
  the polygon, and 1 byte holding the style of the polygon (fill
  style and colour), followed by the coordinates for each vertex,
  two bytes per (x,y) pair.
  The polygon style is encoded as follows:
  Top 6 bits: line width
    0 implies 'filled'
    non-zero implies 'stroked'
  A filled polygon has an implied 'close path' and doesn't need
  a final vertex matching the first.
  A stroked polyon will not imply a final line back to the start
  of the polygon.

  Lowest 2 bits: fill or line colour
    00 = solid
    01 = clear
    10 = 50% solid
    11 = 25% solid

  Note: A filled polygon does not need an explicit closing line
  back to the start. This final line will be used automatically
  by the polygon fill routine.
  A 'stroked' polygon will only draw lines between the vertices
  given.
  Note: A filled polygon needs to be defined in clockwise order.
  Or it might be anti-clockwise. It's one of them.
  =============================================================
*/
#define VERTICES_MAX 40

#define DRAWING_MAXBYTES 400
byte _drawing[ DRAWING_MAXBYTES ] ;

void drawing_init( byte f_polys[] PROGMEM )
{
  // copy the given drawing into the RAM buffer, expanding primitives and translating as neccesary.
  byte *f_poly = &f_polys[0] ;
  int i_poly = 0 ;
  byte polycount = pgm_read_byte( f_poly++ ) ;
  _drawing[i_poly++] = polycount ;
  byte realpolycount = 0 ;

  for ( byte polyindex = 0 ; polyindex < polycount ; polyindex++ ) {

    byte vertex_count = pgm_read_byte( f_poly + 0 ) ;
    byte fill_colour = pgm_read_byte( f_poly + 1 ) ;
    byte linewidth = fill_colour >> 2 ;


    if (linewidth == 0)
    { // this is a standard, filled, unstroked polygon.

      _drawing[ i_poly++ ] = vertex_count ;
      _drawing[ i_poly++ ] = fill_colour ;
      realpolycount ++ ;

      // check that there's room!
      if ((i_poly + 4 * vertex_count) >= DRAWING_MAXBYTES)
      {
        // there isn't room!
        _drawing[0] = 0 ; // mark the buffer as having no polygons.
        return ;
      }

      f_poly += 2 ; // p_poly now points to the first vertex.

      // copy the polygon into the local buffer, transforming on the way in...
      // for the moment, that's just 'convert to fixed point'...
      for ( byte v = 0 ; v < vertex_count ; v++ )
      {
        int x = XSCALE *  pgm_read_byte( f_poly + 0 + (v * 2) ) ; // x
        int y = ( pgm_read_byte( f_poly + 1 + (v * 2) ) + YOFFSET )  ; // y
        _drawing[ i_poly++ ] = (x & 0xff00) >> 8 ;
        _drawing[ i_poly++ ] = (x & 0xff) ;
        _drawing[ i_poly++ ] = (y & 0xff00) >> 8 ;
        _drawing[ i_poly++ ] = (y & 0xff) ;
      }
      f_poly += vertex_count * 2 ;
    }
    else
    { // this is a stroked polygon.
      // copy the vertices across, and generate the deltas for the thickening
      //    Serial.println("About to copy stroked polygon");
      _drawing[ i_poly++ ] = vertex_count ;
      _drawing[ i_poly++ ] = fill_colour ;
      realpolycount ++ ;

      // check that there's room!
      // 4 bytes per vertex, plus 4 bytes between each vertex
      if ((i_poly + 4 + 8 * (vertex_count - 1)) >= DRAWING_MAXBYTES)
      {
        // there isn't room!
        _drawing[0] = 0 ; // mark the buffer as having no polygons.
        return ;
      }

      f_poly += 2 ; // p_poly now points to the first vertex.

      // copy the polygon into the local buffer, transforming on the way in...
      // for the moment, that's just 'convert to fixed point'...
      for ( byte v = 0 ; v < vertex_count - 1 ; v++ )
      {
        int x0 = XSCALE *  pgm_read_byte( f_poly + 0 + (v * 2) ) ; // x
        int y0 = ( pgm_read_byte( f_poly + 1 + (v * 2) ) + YOFFSET ) ; // y
        _drawing[ i_poly++ ] = (x0 & 0xff00) >> 8 ;
        _drawing[ i_poly++ ] = (x0 & 0xff) ;
        _drawing[ i_poly++ ] = (y0 & 0xff00) >> 8 ;
        _drawing[ i_poly++ ] = (y0 & 0xff) ;

        // now read in the next vertex...
        int x1 = XSCALE * pgm_read_byte( f_poly + 2 + (v * 2) ) ; // x
        int y1 = ( pgm_read_byte( f_poly + 3 + (v * 2) ) + YOFFSET )  ; // y
        // and calculate the nx and ny values to use when expanding this line segment...
        int dx = x1 - x0 ;
        int dy = y1 - y0 ;
        // now normalise the vectors...
        float len = sqrt( (float)dx * dx + (float)dy * dy ) ;
        int nx = linewidth * (float)dx / len ;
        int ny = linewidth * (float)dy / len ;
        // now write out the nx and ny values...
        _drawing[ i_poly++ ] = (nx & 0xff00) >> 8 ;
        _drawing[ i_poly++ ] = (nx & 0xff) ;
        _drawing[ i_poly++ ] = (ny & 0xff00) >> 8 ;
        _drawing[ i_poly++ ] = (ny & 0xff) ;
      }
      // now copy the final vertex.
      f_poly += vertex_count * 2 ;
      int x0 = XSCALE * pgm_read_byte( f_poly - 2 ) ; // x
      int y0 = ( pgm_read_byte( f_poly - 1 ) + YOFFSET )  ; // y
      _drawing[ i_poly++ ] = (x0 & 0xff00) >> 8 ;
      _drawing[ i_poly++ ] = (x0 & 0xff) ;
      _drawing[ i_poly++ ] = (y0 & 0xff00) >> 8 ;
      _drawing[ i_poly++ ] = (y0 & 0xff) ;

      //      Serial.println("stroked polygon copied.");
    }
  }
  _drawing[0] = realpolycount ; // put the actual number of polygons at the start of the buffer.

}


void tm_cursor_save( void )
{
#ifndef SIMPLE
  Serial.print( CHAR_ESC ) ;
  Serial.print( 'j' ) ;
#endif
}

const byte f_drawing_arduino[] PROGMEM = {
  5, // number of polygons
  29, 0b0100001, // number of vertices, linewidth + fillcolour
  129, 68, 146, 44, 158, 36, 174, 33, 189, 36, 198, 42,
  206, 52, 209, 67, 206, 82, 198, 93, 188, 99, 174, 102,
  159, 98, 149, 93, 122, 59, 112, 44, 100, 36, 88, 33,
  74, 35, 61, 42, 52, 52, 49, 68, 51, 81, 58, 91,
  70, 99, 83, 102, 99, 99, 112, 90, 129, 68,
  2, 0b00010001, // number of vertices, linewidth + fillcolour
  77, 68, 94, 69,
  2, 0b00010001, // number of vertices, linewidth + fillcolour
  164, 67, 181, 68,
  2, 0b00010001, // number of vertices, linewidth + fillcolour
  172, 76, 171, 59,
  2, 0b00011001, // number of vertices, linewidth + fillcolour
  209, 29, 214, 30
};

const byte f_drawing_radio[] PROGMEM = {
  6, // number of polygons
  11, 1, // number of vertices, colour
  64, 175, 65, 159, 70, 131, 84, 109,
  103, 97, 119, 90, 137, 89, 159, 95,
  182, 119, 192, 145, 195, 175,
  2, 0b00100001, // number of vertices, linewidth + colour
  150, 22, 151, 96,
  8, 0b00001000, // number of vertices, linewidth + colour
  86, 165, 88, 147, 99, 126, 119, 116,
  147, 117, 163, 131, 171, 154, 170, 166,
  2, 0b00001000, // number of vertices, linewidth + colour
  114, 166, 115, 145,
  2, 0b00001000, // number of vertices, linewidth + colour
  144, 166, 145, 147,
  2, 0b00001000, // number of vertices, linewidth + colour
  123, 128, 135, 129
};

const byte f_drawing_cube[] PROGMEM = {
  3, // number of polygons
  24, 1, // number of vertices, fillcolour
  154, 33, 160, 24, 198, 24, 203, 32, 203, 76, 198, 82, 198, 118, 205, 124,
  204, 166, 198, 173, 154, 172, 149, 166, 103, 166, 99, 173, 54, 173, 49, 164,
  49, 129, 57, 120, 57, 79, 47, 74, 47, 33, 57, 25, 99, 25, 105, 34,
  9, 0b00110000, // number of vertices, fillcolour
  111, 60, 147, 61, 172, 83, 173, 112, 149, 138,
  113, 139, 91, 120, 92, 81, 111, 60,
  8, 0, // number of vertices, line width + colour
  131, 114, 117, 106, 113, 95, 118, 87, 130, 94,
  142, 86, 148, 92, 146, 103
};

const byte f_drawing_fire[] PROGMEM = {
  5, // number of polygons
  12, 1, // number of vertices, fillcolour
  115, 125, 90, 108, 79, 96, 76, 83,
  78, 71, 81, 64, 108, 42, 136, 13,
  164, 48, 170, 67, 164, 82, 142, 106,
  10, 1, // number of vertices, fillcolour
  139, 119, 160, 103, 172, 87, 182, 74, 183, 50, 208, 75,
  212, 89, 205, 104, 191, 113, 165, 117,
  9, 1, // number of vertices, fillcolour
  88, 120, 66, 119, 50, 114, 38, 102, 35, 94,
  38, 86, 66, 68, 64, 85, 68, 99,
  2,  0b00100101, // number of vertices, linewidth + colour
  46, 136, 195, 154, //207,154,
  2, 0b00100101, // number of vertices, linewidth + colour
  202, 130, 55, 160 //38,160
};

const byte f_drawing_batsocks[] PROGMEM = {
  1, // number of polygons
  30, 1, // number of vertices, fillcolour
  131, 16, 137, 38, 145, 51, 161, 68, 167, 79, 169, 90, 167, 103, 161, 116,
  151, 125, 156, 132, 158, 144, 157, 156, 164, 176, 150, 167, 144, 169,
  135, 171, 125, 171, 113, 166, 98, 176, 105, 157, 104, 148, 105, 136, 111, 125,
  106, 121, 98, 111, 93, 97, 94, 81, 99, 70, 116, 52, 124, 39
};

const byte f_drawing_atom[] PROGMEM = {
  4, // number of polygons
  13, 0b00010001, // number of vertices, linewidth + fillcolour
  147, 98, 139, 153, 130, 169, 120, 175, 110, 168, 102, 151,
  94, 97, 102, 43, 111, 26, 120, 20, 131, 27, 139, 44, 147, 98,

  13, 0b00010001, // number of vertices, linewidth + fillcolour
  136, 77, 189, 111, 200, 127, 201, 138, 188, 144, 166, 142, 104, 122,
  52, 88, 40, 72, 41, 61, 53, 55, 75, 56, 136, 77,

  13, 0b00010001, // number of vertices, linewidth + fillcolour
  106, 78, 168, 57, 190, 56, 202, 62, 201, 74, 189, 89, 137, 123,
  75, 143, 53, 144, 41, 139, 42, 127, 54, 111, 106, 78,

  7, 1, // number of vertices, linewidth + fillcolour
  131, 99, 127, 106, 118, 107, 113, 102, 114, 94, 119, 90, 127, 91
};

// TICK
const byte f_drawing_tick[] PROGMEM = {
  1, // number of polygons
  23, 1, // number of vertices, fillcolour,
  35, 119, 62, 103, 85, 90, 101, 115, 109, 133, 122, 106, 132, 90, 152, 64, 180, 35, 201, 16, 208, 10, 208, 39, 211, 67,
  213, 77, 202, 81, 188, 92, 167, 111, 135, 149, 116, 174, 110, 183, 86, 157, 66, 139, 48, 126
};
// Broken Heart
const byte f_drawing_brokenheart[] PROGMEM = {
  3, // number of polygons
  23, 1, // number of vertices, fillcolour,
  132, 56, 136, 41, 144, 28, 154, 19, 168, 14, 181, 14, 197, 19, 209, 30, 216, 44, 219, 61, 219, 79, 216, 92,
  208, 106, 191, 131, 182, 149, 176, 164, 173, 175, 169, 162, 176, 129, 156, 135, 168, 105, 145, 105, 155, 69,
  5, 0,    // number of vertices, fillcolour
  173, 35, 185, 33, 194, 52, 184, 58, 174, 47,
  23, 1, // number of vertices, fillcolour
  115, 64, 125, 73, 117, 110, 141, 112, 132, 146, 147, 142, 148, 174, 128, 164, 86, 149, 67, 140, 54, 130, 47, 119,
  44, 110, 42, 101, 42, 93, 43, 84, 48, 69, 56, 57, 65, 50, 76, 46, 86, 46, 98, 48, 108, 54
};

// Cake (it's a lie!) [sorry...]
const byte f_drawing_cake[] PROGMEM = {
  2, // number of polygons
  17, 1, // number of vertices, fillcolour,
  43, 61, 64, 58, 72, 40, 81, 41, 87, 52, 88, 54, 94, 53, 94, 23, 104, 23, 117, 27,
  104, 37, 104, 52, 210, 36, 110, 89, 86, 83, 64, 75, 52, 68,
  19, 1, // number of vertices, fillcolour,
  36, 69, 57, 83, 75, 90, 95, 94, 116, 96, 213, 47, 213, 61, 116, 106, 117, 133, 214, 85,
  214, 99, 117, 143, 117, 177, 214, 121, 214, 135, 117, 187, 79, 173, 52, 159, 35, 146
};

// aperture
const byte f_drawing_aperture[] PROGMEM = {
  2, // number of polygons
  32, 1, // number of vertices, colour
  200, 100, 198, 116, 193, 132, 185, 146, 175, 159, 162, 169, 148, 177, 132, 182,
  116, 184, 99, 182, 83, 177, 69, 169, 56, 159, 46, 146, 38, 132, 33, 116, 32, 100,
  33, 83, 38, 67, 46, 53, 56, 40, 69, 30, 83, 22, 99, 17, 116, 16, 132, 17, 148, 22,
  162, 30, 175, 40, 185, 53, 193, 67, 198, 83,
  24, 0, // number of vertices, colour
  79, 77, 140, 16, 146, 21, 109, 56, 194, 58, 194, 66, 140, 62, 200, 126, 197, 131,
  160, 93, 157, 178, 151, 179, 154, 124, 91, 184, 84, 181, 124, 144, 35, 141,
  36, 135, 91, 138, 32, 75, 35, 69, 72, 108, 74, 22, 81, 20
} ;

// black mesa
const byte f_drawing_blackmesa[] PROGMEM = { 2, // number of polygons
                                             17, 0b0011101, // number of vertices, linewidth + linecolour
                                             200, 96, 194, 125, 178, 148, 155, 162, 127, 168, 99, 161, 77, 146, 62, 123, 57, 95,
                                             63, 67, 78, 44, 100, 30, 130, 24, 157, 30, 180, 46, 195, 69, 200, 96,
                                             8, 1, // number of vertices, fillcolour
                                             100, 133, 100, 96, 180, 96, 193, 117, 178, 147, 150, 168, 107, 168, 64, 133
                                           } ;

// explosion
const byte f_drawing_explosion[] PROGMEM = { 1, // number of polygons,
                                             24, 1, // number of vertices, colour
                                             88, 22, 135, 65, 145, 54, 150, 72, 183, 37, 159, 88, 199, 88, 171, 103, 197, 119, 167, 117, 207, 172, 149, 132, 145, 159, 133, 137,
                                             108, 179, 106, 139, 30, 158, 93, 113, 68, 109, 89, 101, 19, 72, 101, 85, 84, 68, 105, 71
                                           };

// radiation
const byte f_drawing_radiation[] PROGMEM = { 3, // number of polygons
                                             38, 1,
                                             128, 16, 134, 16, 147, 18, 158, 21, 167, 25, 172, 28, 164, 41, 129, 103, 216, 104, 216, 110,
                                             215, 118, 213, 127, 209, 138, 205, 147, 200, 155, 194, 162, 187, 169, 184, 172, 177, 177, 172, 180,
                                             128, 105, 84, 180, 79, 177, 71, 171, 64, 164, 57, 156, 51, 147, 46, 136, 42, 124, 40, 110, 39, 103,
                                             127, 103, 84, 28, 89, 25, 96, 22, 105, 19, 115, 17, 123, 16,
                                             15, 0,
                                             116, 83, 124, 80, 132, 80, 140, 83, 152, 103, 152, 108, 151, 112, 147, 119,
                                             143, 123, 140, 125, 116, 125, 113, 123, 109, 119, 105, 112, 104, 103,
                                             16, 1,
                                             112, 100, 114, 95, 120, 89, 126, 87, 130, 87, 136, 89, 142, 95, 144, 100, 144, 107,
                                             141, 114, 136, 118, 130, 120, 126, 120, 120, 118, 116, 115, 112, 107
                                           };

// Mapping of notes used in the sequence below:
// values nicked from the Arduino's tone() tutorial
const int f_notemap[] PROGMEM = {
  0,  // 0 = no note, e.g. 'pause'.
  220,// 3 A
  233,// 3 Bb
  247,// 4 B
  262,// 4 C
  277,// 4 C#
  294,// 4 D
  311,// 4 D#
  330,// 4 E
  349,// 4 F
  370,// 4 F#
  392,// 4 G
  415,// 4 G#
  440,// 4 A
  466,// 4 A#
  494 // 4 B
} ;

/*
  Music/display sequence.
  Each byte is split into two 4-bit values;
  The upper nybble is the note to play (index into the notes in f_notemap).
  A note of 0 indicates a rest (or delay to the non-musically minded)
  The lower nybble is the duration of the note.
  'Notes' with a duration of 0 are translated differently.
  They carry special information as follows:
  value  meaning
  (hex)
  0    Text to display. null-terminated text follows.
  1    Text delay. 1 byte new text delay (in ms) follows.
  2    Clear 'text' window. Only has effect in TellyMate version.
  3    Clear 'graphics' window. Only has effect in TellyMate version.
  4    Display picture. 1 byte follows, with the following meaning:
                  1   f_drawing_brokenheart
                  2   f_drawing_atom
                  3   f_drawing_cake
                  4   f_drawing_explosion
                  5   f_drawing_blackmesa
                  6   f_drawing_aperture
                  7   f_drawing_radio
                  8   f_drawing_fire
                  9   f_drawing_tick
                  10  f_drawing_radiation
                  11  f_drawing_batsocks
                  12  f_drawing_arduino
  5    Unused
  6    Music delay. 1 byte new '1 beat' time (in ms) follows.
  7    Graphics delay. 1 byte new row delay (in ms) follows.
  8-D  Unused
  E    Loop back to start of sequence.
  F    End of sequence. Stop.
*/

#define QUOTE(name) #name
#define QUOTEHEX( nybble_hi, nybble_lo ) QUOTE( \x##nybble_hi##nybble_lo )
#define NOTE( note, duration ) QUOTEHEX( note,duration )
#define SEMIQUAVER 1
#define QUAVER 2
#define CROTCHET 4
#define MINIM 8
#define QUAVER_DOTTED 3
#define CROTCHET_DOTTED 6
#define MINIM_DOTTED C
#define REST  0
#define A3  1
#define As3 2
#define B3  3
#define C4  4
#define Cs4 5
#define D4  6
#define Ds4 7
#define E4  8
#define F4  9
#define Fs4 A
#define G4  B
#define Gs4 C
#define A4  D
#define As4 E
#define B4  F

#define TEXTDATA( text ) "\x00" text "\x00"

// Non-musical data is indented further for clarity (not that it helps much!)
const byte f_sequence[] PROGMEM =
  // bar 1
  "\x60\x82" // set tempo to 130ms per beat.
  "\x10\x00" // set text rate to fastest
  TEXTDATA( "\x0C" )
  TEXTDATA( "Forms FORM-29827281-12\x0D\x0ATest Assessment Report\x0D\x0A\x0D\x0A" )
  NOTE( REST, MINIM )
  "\x10\x32" // set text rate
  "\x70\x0a" // set graphic rate
  TEXTDATA( "This was a triumph\x0D\x0A" )
  NOTE( G4, QUAVER )
  NOTE( Fs4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( E4, QUAVER )
  // bar 2
  NOTE( Fs4, MINIM )
  NOTE( REST, MINIM )
  // bar 3...
  NOTE( REST, CROTCHET_DOTTED )
  TEXTDATA("I\'m making a note here:")
  NOTE( A3, QUAVER )
  NOTE( G4, QUAVER )
  NOTE( Fs4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( E4, CROTCHET )
  // bar ...4...
  NOTE( Fs4, CROTCHET_DOTTED )
  "\x10\x00" // set text rate
  TEXTDATA(" HUGE ")
  NOTE( D4, CROTCHET )
  TEXTDATA("SUC")
  NOTE( E4, QUAVER )
  TEXTDATA("CESS.")
  NOTE( A3, E )
  // bar ...5
  NOTE( REST, QUAVER )
  "\x10\x5a" // set text rate
  TEXTDATA("\x0D\x0A" "It\'s hard to overstate\x0D\x0A" "my ")
  NOTE( A3, QUAVER )
  // bar 6...
  NOTE( E4, CROTCHET )
  NOTE( Fs4, QUAVER )
  NOTE( G4, CROTCHET_DOTTED )
  NOTE( E4, QUAVER )
  NOTE( Cs4, CROTCHET )
  // bar ...7...
  NOTE( D4, CROTCHET_DOTTED )
  TEXTDATA("sat")
  NOTE( E4, CROTCHET )
  TEXTDATA("is")
  NOTE( A3, QUAVER )
  TEXTDATA("fact")
  NOTE( A3, CROTCHET )
  // bar ...8
  TEXTDATA("ion.\x0D\x0A")
  NOTE( Fs4, E )
  "\x40\x06" // set graphic to aperture
  // bar 9
  NOTE( REST, MINIM )
  NOTE( G4, QUAVER )
  "\x10\x14" // set text rate
  TEXTDATA( "Aperture Science\x0D\x0A" )
  NOTE( Fs4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( E4, QUAVER )
  // bar 10
  NOTE( Fs4, MINIM )
  NOTE( REST, MINIM )
  // bar 11...
  NOTE( REST, CROTCHET_DOTTED )
  "\x10\x28" // set text rate
  TEXTDATA("\x0D\x0A"  "We do what we must ")
  NOTE( A3, QUAVER )
  NOTE( G4, QUAVER )
  NOTE( Fs4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( E4, CROTCHET_DOTTED )
  // bar ...12...
  TEXTDATA("be")
  NOTE( Fs4, QUAVER )
  TEXTDATA("cause ")
  NOTE( D4, CROTCHET_DOTTED )
  TEXTDATA("we ")
  NOTE( E4, QUAVER )
  TEXTDATA("can.")
  NOTE( A3, CROTCHET_DOTTED )
  // bar ...13
  NOTE( REST, B )
  "\x10\x32" // set text rate
  TEXTDATA("\x0D\x0A"  "for the good of all of us")
  NOTE( REST, 1 )
  /// bar 14...
  NOTE( E4, CROTCHET )
  NOTE( Fs4, QUAVER )
  NOTE( G4, CROTCHET_DOTTED )
  NOTE( E4, QUAVER )
  NOTE( Cs4, CROTCHET_DOTTED )
  // bar ...15
  "\x40\x0A" // set graphic to radiation.
  NOTE( D4, QUAVER )
  NOTE( E4, CROTCHET )
  "\x10\x01" // set text rate to 1
  TEXTDATA("\x0D\x0A\x0C"  "Except the ones who are dead")
  NOTE( A3, QUAVER )
  NOTE( D4, QUAVER )
  NOTE( E4, QUAVER )
  // bar 16
  NOTE( F4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( D4, QUAVER )
  NOTE( C4, QUAVER )
  "\x40\x06" // set graphic to aperture
  NOTE( REST, CROTCHET )
  NOTE( A3, QUAVER )
  "\x10\x22"
  TEXTDATA("\x0D\x0A"
           "But there\'s no sense crying over\x0D\x0A"
           "every mistake.")
  NOTE( As3, QUAVER )
  // bar 17
  NOTE( C4, CROTCHET )
  NOTE( F4, CROTCHET )
  NOTE( E4, QUAVER )
  NOTE( D4, QUAVER )
  NOTE( D4, QUAVER )
  NOTE( C4, QUAVER )
  // bar 18
  NOTE( D4, QUAVER )
  NOTE( C4, QUAVER )
  NOTE( C4, CROTCHET )
  NOTE( C4, CROTCHET )
  TEXTDATA("\x0D\x0A"  "You just keep on trying\x0D\x0A"  "\'till you run out of cake.")
  NOTE( A3, QUAVER )
  NOTE( As3, QUAVER )

  // bar 19
  NOTE( C4, CROTCHET )
  NOTE( F4, CROTCHET )
  NOTE( G4, QUAVER )
  NOTE( F4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( D4, QUAVER )
  // bar 20
  NOTE( D4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( F4, CROTCHET )
  NOTE( F4, CROTCHET )
  "\x40\x02" // set graphic to atom
  TEXTDATA("\x0D\x0A"  "and the science gets done")
  NOTE( G4, QUAVER )
  NOTE( A4, QUAVER )
  // bar 21
  NOTE( As4, QUAVER )
  NOTE( As4, QUAVER )
  NOTE( A4, CROTCHET )
  TEXTDATA("\x0D\x0A" "and you make a neat gun")
  NOTE( G4, CROTCHET )
  NOTE( F4, QUAVER )
  NOTE( G4, QUAVER )
  // bar 22
  NOTE( A4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( G4, CROTCHET )
  "\x40\x06" // set graphic to aperture
  NOTE( F4, CROTCHET )
  TEXTDATA("\x0D\x0A"  "for the people who are ")
  NOTE( D4, QUAVER )
  NOTE( C4, QUAVER )
  // bar 23
  NOTE( D4, QUAVER )
  NOTE( F4, QUAVER )
  NOTE( F4, QUAVER )
  NOTE( E4, CROTCHET )
  TEXTDATA("still alive.")
  NOTE( E4, QUAVER )
  NOTE( Fs4, QUAVER )
  NOTE( Fs4, QUAVER )
  // bar 24
  NOTE( REST, MINIM )
  TEXTDATA("\x0d\x0A\x0A\x0A") // a few newlines...
  NOTE( REST, MINIM )
  // bar 25
  NOTE( REST, MINIM )
  TEXTDATA("\x0D\x0A\x0C"  "Forms FORM-55551-5:")
  NOTE( REST, MINIM )
  // bar 26
  TEXTDATA("\x0D\x0A"  "Personnel File Addendum:")
  NOTE( REST, MINIM )
  TEXTDATA("\x0D\x0A\x0D\x0A"  "Dear <<Subject Name Here>>\x0D\x0A")
  NOTE( REST, MINIM )
  // bar 27...
  NOTE( REST, CROTCHET_DOTTED )
  TEXTDATA("\x0D\x0A"  "I\'m not even angry.")
  NOTE( A3, QUAVER )
  NOTE( G4, QUAVER )
  NOTE( Fs4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( E4, QUAVER_DOTTED )
  // bar ...28
  NOTE( Fs4, 7)
  NOTE( REST, MINIM )
  // bar 29...
  NOTE( REST, MINIM )
  TEXTDATA("\x0D\x0A"  "I\'m being so sincere ")
  NOTE( G4, QUAVER )
  NOTE( Fs4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( E4, CROTCHET_DOTTED )
  // bar ...30...
  NOTE( Fs4, QUAVER )
  NOTE( D4, CROTCHET )
  TEXTDATA("right ")
  NOTE( E4, CROTCHET )
  TEXTDATA("now")
  NOTE( A3, CROTCHET_DOTTED )
  // bar ...31
  NOTE( REST, E )
  // bar 32
  TEXTDATA("\x0D\x0A"  "Even though you ")
  NOTE( E4, CROTCHET )
  NOTE( Fs4, QUAVER )
  NOTE( G4, CROTCHET_DOTTED )
  "\x40\x01" // broken heart picture
  NOTE( E4, CROTCHET )
  // bar 33...
  TEXTDATA("broke my heart")
  NOTE( Cs4, CROTCHET )
  NOTE( D4, QUAVER )
  NOTE( E4, CROTCHET_DOTTED )
  TEXTDATA("\x0D\x0A"  "and killed me.")
  NOTE( A3, QUAVER )
  NOTE( A3, CROTCHET )
  "\x40\x04" // explosion picture
  // bar ...34
  NOTE( Fs4, A )
  NOTE( REST, CROTCHET )
  // bar 35
  NOTE( REST, CROTCHET_DOTTED )
  TEXTDATA("\x0D\x0A"  "And tore me to pieces.")
  NOTE( A3, QUAVER )
  NOTE( B4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( G4, QUAVER )
  NOTE( G4, QUAVER )
  // bar 36
  NOTE( A4, CROTCHET )
  NOTE( REST, MINIM_DOTTED )
  // bar 37...
  NOTE( REST, CROTCHET_DOTTED )
  TEXTDATA("\x0D\x0A"  "And threw every piece ")
  NOTE( A3, QUAVER )
  NOTE( B4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( G4, QUAVER )
  NOTE( G4, CROTCHET_DOTTED )
  // bar ...38...
  TEXTDATA("in to ")
  NOTE( A4, QUAVER )
  "\x40\x08" // fire picture
  NOTE( Fs4, CROTCHET_DOTTED )
  TEXTDATA("a fire.")
  NOTE( G4, QUAVER )
  NOTE( D4, CROTCHET_DOTTED )
  // bar ...39
  NOTE( REST, MINIM_DOTTED )
  // bar 40
  NOTE( E4, CROTCHET )
  TEXTDATA("\x0D\x0A"  "As they burned, ")
  NOTE( Fs4, QUAVER )
  NOTE( G4, CROTCHET_DOTTED )
  NOTE( E4, CROTCHET )
  TEXTDATA("it hurt because")
  // bar 41
  NOTE( Cs4, CROTCHET )
  NOTE( D4, QUAVER )
  NOTE( E4, CROTCHET )
  "\x40\x09"  // tick picture
  NOTE( A3, QUAVER )
  TEXTDATA("\x0D\x0A"  "I was so happy for you!")
  NOTE( D4, QUAVER )
  NOTE( E4, QUAVER )
  // bar 42
  NOTE( F4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( D4, QUAVER )
  NOTE( C4, CROTCHET_DOTTED )
  TEXTDATA("\x0D\x0A"  "Now these points of data make a")
  NOTE( A3, QUAVER )
  NOTE( As3, QUAVER )
  // bar 43
  NOTE( C4, CROTCHET )
  NOTE( F4, CROTCHET )
  NOTE( E4, QUAVER )
  NOTE( D4, QUAVER )
  NOTE( D4, QUAVER )
  TEXTDATA("\x0D\x0A"  "beautiful ")
  NOTE( C4, QUAVER )
  // bar 44
  NOTE( D4, QUAVER )
  NOTE( C4, QUAVER )
  TEXTDATA("line")
  NOTE( C4, CROTCHET )
  NOTE( C4, CROTCHET )
  TEXTDATA("\x0D\x0A"  "and we\'re out of beta;")
  NOTE( A3, QUAVER )
  NOTE( As3, QUAVER )
  // bar 45
  NOTE( C4, CROTCHET )
  NOTE( F4, CROTCHET )
  NOTE( G4, QUAVER )
  TEXTDATA("\x0D\x0A"  "we\'re releasing on time.")
  NOTE( F4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( D4, QUAVER )
  // bar 46
  NOTE( D4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( F4, CROTCHET )
  NOTE( F4, CROTCHET )
  "\x40\x04"  // explosion picture
  TEXTDATA("\x0D\x0A"  "So, I\'m GLaD I got burned,")
  NOTE( G4, QUAVER )
  NOTE( A4, QUAVER )
  // bar 47
  NOTE( As4, QUAVER )
  NOTE( As4, QUAVER )
  NOTE( A4, CROTCHET )
  NOTE( G4, CROTCHET )
  "\x40\x02"  // Atom picture
  TEXTDATA("\x0D\x0A"  "Think of all the things we learned")
  NOTE( F4, QUAVER )
  NOTE( G4, QUAVER )
  // bar 48
  NOTE( A4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( G4, QUAVER )
  NOTE( F4, QUAVER )
  NOTE( F4, CROTCHET )
  "\x40\x06"  // Aperture picture
  TEXTDATA("\x0D\x0A"  "For the people who are ")
  NOTE( D4, QUAVER )
  NOTE( C4, QUAVER )
  // bar 49...
  NOTE( D4, QUAVER )
  NOTE( F4, QUAVER )
  NOTE( F4, QUAVER )
  NOTE( E4, CROTCHET )
  TEXTDATA("still alive.")
  NOTE( E4, QUAVER )
  NOTE( Fs4, QUAVER )
  NOTE( Fs4, A )
  // bar ...50
  NOTE( REST, MINIM )
  // bar 51
  NOTE( REST, MINIM )
  TEXTDATA("\x0D\x0A\x0A\x0A\x0C"  "Forms FORM-55551-6"
           "\x0D\x0A\x0D\x0A"  "[Personnel File Addendum Addendum:"
           "\x0D\x0A"          "One Last Thing:]")
  NOTE( REST, MINIM )
  // bar 52
  NOTE( REST, MINIM )
  NOTE( REST, MINIM )
  // bar 53...
  NOTE( REST, MINIM )
  TEXTDATA("\x0D\x0A\x0A"  "Go ahead and leave me.")
  NOTE( G4, QUAVER )
  NOTE( Fs4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( E4, CROTCHET )
  // bar ...54
  NOTE( Fs4, CROTCHET_DOTTED )
  NOTE( REST, MINIM )
  // bar 55...
  NOTE( REST, CROTCHET_DOTTED )
  TEXTDATA("\x0D\x0A"  "I think I prefer ")
  NOTE( A3, QUAVER )
  NOTE( G4, QUAVER )
  NOTE( Fs4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( E4, CROTCHET_DOTTED )
  // bar ...56...
  TEXTDATA("to stay ")
  NOTE( Fs4, QUAVER )
  NOTE( D4, CROTCHET_DOTTED )
  TEXTDATA("inside.")
  NOTE( E4, QUAVER )
  NOTE( A3, A )
  // bar ...57
  NOTE( REST, MINIM )
  // bar 58
  TEXTDATA("\x0D\x0A"  "Maybe you'll find someone else")
  NOTE( E4, CROTCHET )
  NOTE( Fs4, QUAVER )
  NOTE( G4, CROTCHET_DOTTED )
  NOTE( E4, CROTCHET )
  // bar 59...
  NOTE( Cs4, CROTCHET )
  NOTE( D4, QUAVER )
  NOTE( E4, CROTCHET )
  NOTE( REST, QUAVER )
  TEXTDATA("\x0D\x0A"  "to help you")
  NOTE( A3, QUAVER )
  NOTE( A3, CROTCHET )
  // bar ...60
  NOTE ( Fs4, A )
  NOTE( REST, CROTCHET  )
  // bar 61...
  NOTE( REST, CROTCHET  )
  "\x40\x05"  // black mesa picture
  NOTE( REST, CROTCHET  )
  TEXTDATA("\x0D\x0A"  "Maybe Black Mesa.")
  NOTE( B4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( G4, QUAVER )
  NOTE( G4, CROTCHET )
  // bar ...62
  NOTE( A4, CROTCHET )
  NOTE( REST, A )
  // bar 63...
  NOTE( REST, MINIM )
  TEXTDATA("\x0D\x0A"  "THAT ")
  NOTE( B4, QUAVER )
  TEXTDATA("WAS ")
  NOTE( A4, QUAVER )
  TEXTDATA("A ")
  NOTE( G4, QUAVER )
  TEXTDATA("JOKE ")
  NOTE( G4, CROTCHET_DOTTED )
  // bar ...64...
  TEXTDATA("HA ")
  NOTE( A4, QUAVER )
  TEXTDATA("HA! ")
  NOTE( Fs4, CROTCHET_DOTTED )
  TEXTDATA("FAT ")
  NOTE( G4, QUAVER )
  TEXTDATA("CHANCE!")
  NOTE( D4, CROTCHET_DOTTED )
  "\x40\x03"  // cake picture
  // bar ...65
  "\x60\x87" // set tempo to 135ms per beat. (slightly slower)
  NOTE( REST, MINIM_DOTTED )
  // bar 66
  TEXTDATA("\x0D\x0A"  "Anyway this cake is great!")
  NOTE( E4, CROTCHET )
  NOTE( Fs4, QUAVER )
  NOTE( G4, CROTCHET_DOTTED )
  NOTE( E4, CROTCHET )
  // bar 67
  NOTE( Cs4, CROTCHET )
  NOTE( D4, QUAVER )
  TEXTDATA("\x0D\x0A"  "It\'s so delicious and moist")
  NOTE( E4, CROTCHET )
  NOTE( A3, QUAVER )
  NOTE( D4, QUAVER )
  NOTE( E4, QUAVER )
  // bar 68
  NOTE( F4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( D4, QUAVER )
  "\x40\x07"  // GLaDos picture
  NOTE( C4, CROTCHET_DOTTED )
  TEXTDATA("\x0D\x0A"  "Look at me still ")
  NOTE( A3, QUAVER )
  NOTE( As3, QUAVER )
  // bar 69
  NOTE( C4, CROTCHET )
  NOTE( F4, CROTCHET )
  TEXTDATA("talking \x0D\x0Awhen there\'s science to do")
  NOTE( E4, QUAVER )
  NOTE( D4, QUAVER )
  NOTE( D4, QUAVER )
  NOTE( C4, QUAVER )
  // bar 70"
  NOTE( D4, QUAVER )
  NOTE( C4, QUAVER )
  NOTE( C4, CROTCHET )
  "\x40\x06"    // aperture picture
  NOTE( C4, CROTCHET )
  TEXTDATA("\x0D\x0A"    "When I look out there")
  NOTE( A3, QUAVER )
  NOTE( As3, QUAVER )
  // bar 71
  NOTE( C4, CROTCHET )
  NOTE( F4, CROTCHET )
  NOTE( G4, QUAVER )
  TEXTDATA("\x0D\x0A"    "it makes me GLaD I\'m not you.")
  NOTE( F4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( D4, QUAVER )
  // bar 72
  NOTE( D4, QUAVER )
  NOTE( E4, QUAVER )
  NOTE( F4, CROTCHET )
  "\x40\x02"    // atom picture
  NOTE( F4, CROTCHET )
  TEXTDATA("\x0D\x0A"    "I\'ve experiments to run;")
  NOTE( G4, QUAVER )
  NOTE( A4, QUAVER )
  // bar 73
  NOTE( As4, QUAVER )
  NOTE( As4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( G4, QUAVER )
  "\x40\x04"    // explosion picture
  NOTE( G4, CROTCHET )
  TEXTDATA("\x0D\x0A"    "there is research to be done")
  NOTE( F4, QUAVER )
  NOTE( G4, QUAVER )
  // bar 74
  NOTE( A4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( G4, QUAVER )
  NOTE( F4, QUAVER )
  "\x40\x06"    // aperture picture
  NOTE( F4, CROTCHET )
  TEXTDATA("\x0D\x0A"    "on the people who are")
  NOTE( D4, QUAVER )
  NOTE( C4, QUAVER )
  // bar 75...
  NOTE( D4, QUAVER )
  NOTE( F4, QUAVER )
  NOTE( F4, QUAVER )
  TEXTDATA("\x0D\x0A"    "still alive.")
  NOTE( E4, CROTCHET )
  NOTE( E4, QUAVER )
  NOTE( Fs4, QUAVER )
  NOTE( Fs4, A )
  // bar...76
  "\x60\x8c" // set tempo to 140ms per beat. (slightly slower)
  NOTE( REST, QUAVER )
  TEXTDATA("\x0D\x0A\x0A\x0APS: ")
  NOTE( REST, QUAVER )
  TEXTDATA("And believe me, I am")
  NOTE( A4, QUAVER )
  NOTE( A4, QUAVER )
  // bar 77...
  NOTE( B4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( Fs4, QUAVER )
  TEXTDATA("\x0D\x0A"  "still alive.")
  NOTE( D4, CROTCHET )
  NOTE( E4, QUAVER )
  NOTE( Fs4, QUAVER )
  NOTE( A4, CROTCHET_DOTTED )
  // bar ...78
  "\x60\x91" // set tempo to 145ms per beat. (slightly slower)
  NOTE( REST, QUAVER )
  "\x10\x01" // set text rate to 1
  TEXTDATA("\x0D\x0A\x0A"   "PPS: ")
  "\x10\x22" // set text rate
  NOTE( REST, CROTCHET )
  TEXTDATA("I\'m doing science and I\'m")
  NOTE( A4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( A4, QUAVER )
  // bar 79...
  NOTE( B4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( Fs4, QUAVER )
  TEXTDATA("\x0D\x0A"  "still alive.")
  NOTE( D4, CROTCHET )
  NOTE( G4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( A4, CROTCHET_DOTTED )
  // bar ...80
  "\x60\x96" // set tempo to 150ms per beat. (slightly slower)
  NOTE( REST, QUAVER )
  "\x10\x01" // set text rate to 1
  TEXTDATA("\x0D\x0A\x0A"   "PPPS: ")
  "\x10\x22" // set text rate
  NOTE( REST, CROTCHET )
  TEXTDATA("I feel FANTASTIC and I\'m")
  NOTE( A4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( A4, QUAVER )
  // bar 81...
  NOTE( B4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( Fs4, QUAVER )
  TEXTDATA("\x0D\x0A"    "still alive.")
  NOTE( D4, CROTCHET )
  NOTE( G4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( A4, CROTCHET_DOTTED )
  // bar ...82
  "\x60\x9b" // set tempo to 155ms per beat. (slightly slower)
  NOTE( REST, CROTCHET )
  "\x10\x01" // set text rate to 1
  TEXTDATA("\x0D\x0A\x0A"   "FINAL THOUGHT:")
  "\x10\x22" // set text rate
  NOTE( REST, CROTCHET )
  TEXTDATA("\x0D\x0A"  "While you\'re dying I\'ll be")
  NOTE( A4, QUAVER )
  NOTE( A4, QUAVER )
  // bar 83...
  NOTE( B4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( Fs4, QUAVER )
  NOTE( D4, CROTCHET )
  TEXTDATA("\x0D\x0A"    "still alive.")
  NOTE( G4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( A4, CROTCHET_DOTTED )
  //bar ...84
  "\x60\xa0" // set tempo to 160ms per beat. (slightly slower)
  NOTE( REST, QUAVER )
  "\x10\x01" // set text rate to 1
  TEXTDATA("\x0D\x0A\x0A"   "FINAL THOUGHT PS:")
  "\x10\x22" // set text rate
  NOTE( REST, CROTCHET )
  TEXTDATA("\x0D\x0A"  "And when you\'re dead I will be")
  NOTE( A4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( A4, QUAVER )
  // bar 85...
  NOTE( B4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( Fs4, QUAVER )
  NOTE( D4, CROTCHET )
  TEXTDATA("\x0D\x0A"    "still alive.")
  NOTE( G4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( A4, CROTCHET_DOTTED )
  // bar ...86...
  "\x60\xa5" // set tempo to 165ms per beat. (slightly slower)
  NOTE( REST, QUAVER )
  TEXTDATA("\x0D\x0A\x0A")
  NOTE( REST, CROTCHET )
  TEXTDATA("STILL ALIVE")
  NOTE( G4, QUAVER )
  NOTE( A4, QUAVER )
  NOTE( A4, CROTCHET_DOTTED )
  // bar ...87...
  NOTE( REST, CROTCHET_DOTTED )
  NOTE( G4, QUAVER )
  NOTE( Fs4, QUAVER )
  NOTE( Fs4, A )
  // bar ...88
  NOTE( REST, MINIM )
  // delays...
  NOTE( REST, F )
  TEXTDATA("\x0D\x0A\x0A\x0A\x0A\x0A\x0C"
           "THANK YOU FOR PARTICIPATING IN THIS"
           "\x0D\x0A" "ENRICHMENT CENTER ACTIVITY!")
  NOTE( REST, F )
  NOTE( REST, F )
  NOTE( REST, F )
  NOTE( REST, F )
  NOTE( REST, F )
  TEXTDATA( "\x0d\x0A\x0A\x0A\x0A\x0A\x0A\x0A\x0A\x0C")
  NOTE( REST, F )
  "\x10\x14" // set text rate to 20ms between characters (50cps)
  TEXTDATA( "Credits:"
            "\x0d\x0a" "Portal - Still Alive"
            "\x0d\x0a")
  NOTE( REST, F )
  TEXTDATA( "\x0d\x0a" "Source material:" )
  NOTE( REST, F )
  TEXTDATA( "\x0d\x0a\x0a" "  Graphics: Valve"
            "\x0d\x0a" "  Music:    Jonathan Coulton")
  NOTE( REST, F )
  TEXTDATA( "\x0d\x0a\x0a" "Derivation:")
  NOTE( REST, F )
  TEXTDATA( "\x0d\x0a\x0a" "  Platform: Arduino"
            "\x0d\x0a" "  Display:  Batsocks TellyMate Shield"
            "\x0d\x0a" "  Code:     Nigel Batten" )
  NOTE( REST, F )
  NOTE( REST, F )
  NOTE( REST, F )
  TEXTDATA( "\x0d\x0a\x0a" "No arduinos were euthanised during"
            "\x0d\x0a" "development of this demonstration." )
  NOTE( REST, F )
  NOTE( REST, F )
  TEXTDATA( "\x0d\x0a\x0a" "The University of Scranton" )
  NOTE( REST, F )
  TEXTDATA( "\x0d\x0a\x0a" "Thomas Swartz" )
  NOTE( REST, F )
  NOTE( REST, F )
  "\x40\x0c" // arduino picture
  NOTE( REST, F )
  NOTE( REST, F )
  TEXTDATA( "\x0d\x0a\x0c" "Good bye." )
  NOTE( REST, F )
  NOTE( REST, F )
  NOTE( REST, F )
  NOTE( REST, F )
  NOTE( REST, F )
  NOTE( REST, F )
  NOTE( REST, F )
  NOTE( REST, F )
  NOTE( REST, F )
  NOTE( REST, F )
  NOTE( REST, F )
  // end of tune, start from the beginning again.
  "\xe0"
  ;

// position of text and graphics areas on the tellymate screen.
// it's assumed that the text area is at the bottom of the screen.
#define TEXT_TOPLINE        21
#define TEXT_BOTTOMLINE     24

// size of the buffer of text to 'teletype' out.
#define TEXT_BUFFERSIZE     100

#define GRAPHIC_TOPLINE     0
#ifdef USE_TELLYMATE
#define GRAPHIC_BOTTOMLINE  20
#else
#define GRAPHIC_BOTTOMLINE  24
#endif

class graphic {
  private:
    enum renderstyle {
      rs_wipe_down = 0,
      rs_wipe_up = 1,
      rs_random = 2
    };
    renderstyle _style ;
    byte _nextrow  ;
    byte *_drawing  ;
    byte _rate ;
    unsigned long _nextevent ;
    byte _drawn[ GRAPHIC_BOTTOMLINE - GRAPHIC_TOPLINE + 1] ;
  public:
    byte complete ;

    graphic() : _nextrow( GRAPHIC_BOTTOMLINE + 1 ), _style( rs_wipe_down ), _drawing( f_drawing_tick ), _nextevent( millis() + 1000 ), complete( 1 ), _rate( 2 ) {}

    void push( byte *ptr_drawing PROGMEM ) {
      switch (_style) {
        case rs_wipe_down:
          _nextrow = GRAPHIC_TOPLINE ;
          break ;
#ifndef SIMPLE
        case rs_wipe_up:
          _nextrow = GRAPHIC_BOTTOMLINE ;
          break ;
        case rs_random:
          for ( byte row = 0 ; row <= (GRAPHIC_BOTTOMLINE - GRAPHIC_TOPLINE) ; row++ ) {
            _drawn[row] = 0 ;
          }
          _nextrow = random( GRAPHIC_TOPLINE, GRAPHIC_BOTTOMLINE + 1 ) ;
          _drawn[_nextrow] = 1 ;
          break ;
#endif
      }
      _drawing = ptr_drawing ;
      complete = 0 ;
      drawing_init( ptr_drawing ) ;
    }

    void out( void ) {
      if (!complete)
      {
        tm_cursor_save() ;
        tm_cursor_show( false ) ;
        draw_drawing_row( _nextrow - GRAPHIC_TOPLINE ) ;
        tm_cursor_restore() ;
        tm_cursor_show( true ) ;
        switch (_style) {
          case rs_wipe_down:
            if (_nextrow == GRAPHIC_BOTTOMLINE) {
              complete = 1 ;
#ifndef SIMPLE
              _style = rs_wipe_up ;
#endif
            }
            _nextrow++ ;
            break ;
#ifndef SIMPLE
          case rs_wipe_up:
            if (_nextrow == GRAPHIC_TOPLINE) {
              complete = 1 ;
              _style = rs_random ;
            }
            _nextrow-- ;
            break ;
          case rs_random:
            boolean found = false;
            for ( byte row = 0 ; row <= (GRAPHIC_BOTTOMLINE - GRAPHIC_TOPLINE) ; row++ ) {
              if (!_drawn[row]) {
                found = true ;
                break;
              }
            }
            if (found) {
              // there is at least one row that hasn't been drawn...
              found = false ;
              while ( !found ) {
                _nextrow =  random( GRAPHIC_TOPLINE, GRAPHIC_BOTTOMLINE + 1 ) ;
                if (!_drawn[_nextrow]) found = true ;
              }
              _drawn[_nextrow] = 1 ;
            } else {
              complete = 1 ;
              _style = rs_wipe_down ;
            }
            break ;
#endif
        }
      }
    }

    void clear( void ) {
      _nextrow = GRAPHIC_BOTTOMLINE + 1 ;
      tm_cursor_save() ;
      tm_move( GRAPHIC_BOTTOMLINE, SCANLINE_WIDTH - 1 ) ;
      tm_clear_to_top() ;
      tm_cursor_restore() ;
    }

    void setrate( byte rate ) {
#ifdef SIMPLE
      _rate = 0 ;
#else
      _rate = rate ;
#endif
    }

    void tick( void ) {
      unsigned long timenow = millis() ;
      if (timenow < _nextevent) return ;
      _nextevent = timenow + _rate ;
      out() ;
    }
};

#ifndef SIMPLE
class textwindow {
  private:
    static const byte ROW_TOP = 21 ;
    static const byte ROW_BOTTOM = 24 ;
    static const byte COL_LEFT = 0 ;
    static const byte COL_RIGHT = 37 ;
    static const byte AUTO_CRLF = 1 ;
    byte buffer[ ROW_BOTTOM - ROW_TOP + 1 ][ (COL_RIGHT - COL_LEFT) + 1 ] ;
    byte cursor_row ;
    byte cursor_col ;

  public:

    textwindow( void ) {
      clear();
    }

    void refresh( void ) {
      tm_cursor_save() ;
      tm_cursor_show( false ) ;
      for ( byte row = 0 ; row <= (ROW_BOTTOM - ROW_TOP) ; row++ ) {
        tm_move( ROW_TOP + row, COL_LEFT ) ;
        for ( byte col = 0 ; col <= (COL_RIGHT - COL_LEFT) ; col++ ) {
          Serial.write( buffer[ row ][ col ] ) ;
        }
      }
      tm_cursor_show( true ) ;
      tm_cursor_restore() ;
    }

  private:
    void _scrollup( void ) {
      for ( byte row = 0 ; row < (ROW_BOTTOM - ROW_TOP) ; row++ ) {
        for ( byte col = 0 ; col <= (COL_RIGHT - COL_LEFT ) ; col++ ) {
          buffer[ row ][ col ] = buffer[ row + 1 ][ col ] ;
        }
      }
      for ( byte col = 0 ; col <= (COL_RIGHT - COL_LEFT ) ; col++ ) {
        buffer[ ROW_BOTTOM - ROW_TOP ][ col ] = CHAR_SPACE ;
      }
      refresh();
    }

  public:
    void write( char c ) {
      if ( c >= 32 ) { // this is a normal character.

        if ( (cursor_col + 1) > COL_RIGHT ) { // would this be the last character in the row?
          if (AUTO_CRLF) {
            // we need to do a newline (CRLF)
            if ((cursor_row + 1) > ROW_BOTTOM ) {
              // we're already on the bottom row... we need to scroll...
              buffer[ cursor_row - ROW_TOP ][ cursor_col - COL_LEFT ] = c ;
              _scrollup();
              tm_move( cursor_row - 1, cursor_col ) ;
              cursor_col = COL_LEFT ;
              tm_move( cursor_row, cursor_col ) ;
            }
          } else {
            buffer[ cursor_row - ROW_TOP ][ cursor_col - COL_LEFT ] = c ;
            Serial.write( c ) ;
            tm_move( cursor_row, cursor_col ) ;
          }
        } else {
          // this character will not cause a CRLF
          buffer[ cursor_row - ROW_TOP ][ cursor_col - COL_LEFT ] = c ;
          Serial.write( c ) ;
          cursor_col++ ;
        }
      }
      if ( c == 0x0D ) {
        // Carriage return
        cursor_col = COL_LEFT ;
        if ( COL_LEFT == 0 ) {
          Serial.write( c ) ;
        } else {
          tm_move( cursor_row, cursor_col ) ;
        }
      }
      if ( c == 0x0A ) {
        // Line feed
        if ((cursor_row + 1) > ROW_BOTTOM) {
          // we're already on the bottom row... we need to scroll...
          _scrollup();
          // no need to move the cursor. It will stay in the same place.
        } else {
          // We're not on the bottom row yet. No need to scroll.
          Serial.write( c ) ;
          cursor_row++ ;
        }
      }
      if ( c == 0x0C ) {
        // Form Feed
        clear();
      }
    }

    void clear( void ) {
      for ( byte row = 0 ; row <= (ROW_BOTTOM - ROW_TOP) ; row++ ) {
        for ( byte col = 0 ; col <= (COL_RIGHT - COL_LEFT ) ; col++ ) {
          buffer[ row ][ col ] = CHAR_SPACE ;
        }
      }
      cursor_row = ROW_TOP ;
      cursor_col = COL_LEFT ;
      tm_move( ROW_TOP, COL_LEFT ) ;
      refresh();
    }
};

static textwindow m_textwindow ;
#endif

class teletype {
  private:
    byte _buffer[ TEXT_BUFFERSIZE ] ;
    byte _readpos ;
    byte _writepos ;
    byte _empty ;
    unsigned long _nextevent ;
    byte _rate;

    void _out( void ) {
      if (_empty) return ;
      char c = _buffer[ _readpos ] ;

#ifdef SIMPLE
      if (c != 0x0C) Serial.write( c ) ;
#else
      m_textwindow.write( c ) ;
#endif

      byte nextpos = _readpos + 1 ;
      if (nextpos == TEXT_BUFFERSIZE) nextpos = 0 ;
      if (nextpos == _writepos) _empty = 1 ;
      _readpos = nextpos ;
    }

  public:
    teletype() : _readpos( 0 ), _writepos( 0 ), _empty( 1 ), _rate( 250 ), _nextevent( millis() + 1000 ) {}

    void push( char c ) {
      _buffer[ _writepos ] = c ;
      byte nextpos = _writepos + 1 ;
      if (nextpos == TEXT_BUFFERSIZE) nextpos = 0 ;
      // note: if the buffer gets filled, the writepos isn't updated.
      if (nextpos != _readpos) _writepos = nextpos ;
      _empty = 0 ;
    }

    void tick( void ) {
      unsigned long timenow = millis() ;
      if (timenow < _nextevent) return ;
      _nextevent = timenow + _rate ;
      _out() ;
    }

    void setrate( byte rate ) {
      _rate = rate ;
    }

    void clear( void ) {
      tm_move( TEXT_TOPLINE, 0 ) ;
      tm_clear_to_end() ;
    }
} ;

static graphic m_graphic ;
static teletype m_text ;

/* =====================*/
void playMusic() {
  unsigned long timenow = millis() ;

  if (timenow >= m_nextevent)
  {
    // read + decode the next event...
    byte event = pgm_read_byte( m_P_event ) ;

    // event is 2 x 4-bit nybbles:    note,duration
    byte note = (event & 0xf0) >> 4 ;
    byte duration_beats = event & 0x0f ;

    // if duration is 0, then it's not a note or rest, so needs more decoding.
    if ( duration_beats != 0 )
    {
      if (timenow >= m_nextevent)
      {
        m_P_event++ ; // increment the music pointer.
        // this is a normal, non-zero note.
        // simply start the tone, set the 'next event' time and get out.

        int duration_millis = ((int)duration_beats * m_millis_per_beat ) ;
        m_nextevent = timenow + duration_millis ;

        if (note > 0) // a 0 note is a pause.
        {
          int freq = pgm_read_word( &f_notemap[ note ] ) ;
          noTone(50);
          tone( 50, freq, duration_millis - 20 ) ;
          //        tone1.play( freq, duration_millis - 20 ) ;
        }
      }
    }
    else
    {
      m_P_event++ ; // increment the music pointer.
      // This is a special 'note' (e.g. it's NOT a note...)
      byte value ;
      switch ( note )
      {
        case 0x0 : // Text to display. null-terminated text follows.
          while ( char c = pgm_read_byte( m_P_event++ )) {
            m_text.push( c ) ;
          }
          break ;
        case 0x01 : // Text delay. 1 byte new text delay (in ms) follows.
          value = pgm_read_byte( m_P_event++ ) ;
          m_text.setrate( value ) ;
          break ;
        case 0x02 : // Clear text window. (not implemented)
          break ;
        case 0x03 : // Clear graphics window. (not implemented)
          break ;
        case 0x04 : // display picture
          {
            byte *picture PROGMEM = f_drawing_tick ;
            value = pgm_read_byte( m_P_event++ ) ;

            switch ( value )
            {
              case 1 :
                picture = f_drawing_brokenheart ;
                break ;
              case 2 :
                picture = f_drawing_atom ;
                break ;
              case 3 :
                picture = f_drawing_cake ;
                break ;
              case 4 :
                picture = f_drawing_explosion ;
                break ;
              case 5 :
                picture = f_drawing_blackmesa ;
                break ;
              case 6 :
                picture = f_drawing_aperture ;
                break ;
              case 7 :
                picture = f_drawing_radio ;
                break ;
              case 8 :
                picture = f_drawing_fire ;
                break ;
              case 9 :
                picture = f_drawing_tick ;
                break ;
              case 10 :
                picture = f_drawing_radiation ;
                break ;
              case 11 :
                picture = f_drawing_batsocks ;
                break ;
              case 12 :
                picture = f_drawing_arduino ;
                break ;
            }
            m_graphic.push( picture ) ;
          }
          break ;
        case 0x06 : // Music delay. 1 byte new 'beat' time (in ms) follows.
          value = pgm_read_byte( m_P_event++ ) ;
          m_millis_per_beat = value ;
          break ;
        case 0x07 : // graphic delay 1 byte new row delay (in ms) follows.
          value = pgm_read_byte( m_P_event++ ) ;
          m_graphic.setrate( value ) ;
          break ;
        case 0x0e : // loop back to start of sequence.
          m_P_event = f_sequence ;
          break ;
        case 0x0f : // End of sequence. Stop.
          while (1) {
            /* loop forever */
          } ;
      }
    }
  }

  // text cannot be output if we're in the simple mode and
  // the graphics display is being output...
  // (e.g. don't try outputting text whilst the drawing is being output)
#ifdef SIMPLE
  if (m_graphic.complete)
#endif
    m_text.tick() ;
  if (timenow + 50 < m_nextevent) m_graphic.tick() ;

}

void musicSetup() {
  Serial.begin( BAUD_RATE );
  delay( 500 ) ;
#ifdef USE_TELLYMATE
  tm_set_screen_fontbank( GRAPHICS_FONTBANK ) ;
#endif
  m_P_event = f_sequence ;
  m_nextevent = millis() + 2000 ;
}


// Each row of text is sub-sampled 8 times.
//#define SCANLINES_PER_ROW 8
//#define YDIVISOR 1
#define SCANLINES_PER_ROW 4
#define SCANLINES_YSCALE 2

void do_intersect( int x0, int y0, int x1, int y1, int y, float slope, byte &crossing_count, int segments[] ) {
  if ( ( (y0 <= y ) && ( y1 >  y ) ) ||
       (( y0 >  y ) && ( y1 <= y ) ) ) {
    int crossing = (int)(x0  + slope * ( y - y0 ) );
    if (crossing < 0) crossing = 0 ;
    if (y1 > y) crossing = -crossing ;
    segments[crossing_count] = crossing ;
    crossing_count++;
  }
}

// y, v, j, i were ints.
void poly_scanline_intersections( uint8_t y, byte *poly , byte &crossing_count, int segments[] )
{
  // calculate line intersections...
  crossing_count = 0;
  byte vertex_count = *poly++ ;
  byte fillstyle = *poly++ ;
  byte linewidth = fillstyle >> 2 ;

  if (linewidth)
  { // stroked polygon (e.g. draw lines)
    for (uint8_t v = 0 ; v < (vertex_count - 1) ; v++ )
    {
      int base = v * 8 ;
      int x0 = (poly[base + 0] << 8) + poly[base + 1] ;
      int y0 = (poly[base + 2] << 8) + poly[base + 3] ;
      int nx = (poly[base + 4] << 8) + poly[base + 5] ;
      int ny = (poly[base + 6] << 8) + poly[base + 7] ;
      base += 8 ;
      if (v == (vertex_count - 1)) base = 0 ;
      int x1 = (poly[base + 0] << 8) + poly[base + 1] ;
      int y1 = (poly[base + 2] << 8) + poly[base + 3] ;

      //Serial.print("stroke vertex=");
      //Serial.print((int)v);
      //Serial.print(": nx=");
      //Serial.print(nx) ;
      //Serial.print(", ny= " ) ;
      //Serial.println(ny) ;

      if ( (( (y0 <= y + linewidth ) && ( y1 >  y - linewidth ) ) ||
            (( y0 >  y - linewidth) && ( y1 <= y + linewidth) ) ) )
      {
        int dx = x1 - x0 ;
        int dy = y1 - y0 ;
        // now normalise the vectors...
        /*
          float len = sqrt( (float)dx * dx + (float)dy * dy ) ;
          int nx = linewidth * (float)dx / len ;
          int ny = linewidth * (float)dy / len ;
        */

        int xA = x0 - nx - ny ;
        int yA = y0 - ny + nx ;

        int xB = x0 - nx + ny ;
        int yB = y0 - ny - nx ;

        int xC = x1 + nx + ny ;
        int yC = y1 + ny - nx ;

        int xD = x1 + nx - ny ;
        int yD = y1 + ny + nx ;
        /*
                Serial.print("ABCD =");
                Serial.print("( ");
                Serial.print(xA);
                Serial.print(", ");
                Serial.print(yA);
                Serial.print("), (");
                Serial.print(xB);
                Serial.print(", ");
                Serial.print(yB);
                Serial.print("), (");
                Serial.print(xC);
                Serial.print(", ");
                Serial.print(yC);
                Serial.print("), (");
                Serial.print(xD);
                Serial.print(", ");
                Serial.print(yD);
                Serial.println(").");
        */

        float slope1 ;
        float slope2 ;
        if (dx == 0) {
          slope1 = 0.0;
          slope2 = 1.0;
        }
        if (dy == 0) {
          slope1 = 1.0;
          slope2 = 0.0;
        }
        if ((dx != 0) && (dy != 0))
        { // calculate the slopes
          slope1 = -(float)dy / dx;
          slope2 = (float)dx / dy;
        }
        do_intersect( xA, yA, xB, yB, y, slope1, crossing_count, segments ) ;
        do_intersect( xB, yB, xC, yC, y, slope2, crossing_count, segments ) ;
        do_intersect( xC, yC, xD, yD, y, slope1, crossing_count, segments ) ;
        do_intersect( xD, yD, xA, yA, y, slope2, crossing_count, segments ) ;
      }
    }
  }
  else
  { // simple filled polygon
    for (uint8_t v = 0 ; v < vertex_count ; v++ )
    {
      int base = v * 4 ;
      int x0 = (poly[base + 0] << 8) + poly[base + 1] ;
      int y0 = (poly[base + 2] << 8) + poly[base + 3] ;
      base += 4 ;
      if (v == (vertex_count - 1)) base = 0 ;
      int x1 = (poly[base + 0] << 8) + poly[base + 1] ;
      int y1 = (poly[base + 2] << 8) + poly[base + 3] ;

      if ( (( y0 <= y ) && ( y1 >  y ) ) ||
           (( y0 >  y ) && ( y1 <= y ) ) )
      {
        // calculate the crossing point...
        // first calculate the slope...
        float slope ;
        int dx = x1 - x0 ;
        int dy = y1 - y0 ;
        if (dx == 0) slope = 0.0;
        if (dy == 0) slope = 1.0;
        if ((dx != 0) && (dy != 0))
        { // calculate the inverse slope
          slope = (float)dx / dy;
        }

        int crossing = (int)(x0  + slope * ( y - y0 ) );
        if (crossing < 0) crossing = 0;
        if (y1 > y) crossing = -crossing ;
        segments[crossing_count] = crossing ;
        crossing_count++;
      }
    }
  }

  // arrange intersections in order (ignoring sign)...
  for (uint8_t j = 0 ; j < crossing_count - 1 ; j++ ) /*- Arrange x-intersections in order -*/
  {
    for ( uint8_t i = 0 ; i < crossing_count - 1 ; i++ )
    {
      if ( abs(segments[i]) > abs(segments[i + 1]) )
      {
        int temp = segments[ i ];
        segments[ i ] = segments[ i + 1 ];
        segments[ i + 1 ] = temp;
      }
    }
  }
}





void draw_drawing_row( byte row )
{
  byte buffer[ SCANLINE_WIDTH ]; // buffer to hold a single scanline.
  int segments[ VERTICES_MAX * 2 ]; // buffer to hold polygon segments for a scanline
  byte sum[ SCANLINE_WIDTH ];
  //int n,j,k,gd,gm,dy,dx;
  //int x;

  //#ifdef USE_TELLYMATE
  //  tm_cursor_show( false ) ;
  //  Serial.print( CHAR_ESC ) ;
  //  Serial.print( 'E' ) ;
  //#endif

  // clear the 'sum of pixels set'
  for ( byte b = 0 ; b < SCANLINE_WIDTH ; b++ ) sum[b] = 0;

  // each row is built from a number of scanlines. 8 is typical.
  for ( byte slice = 0 ; slice < SCANLINES_PER_ROW ; slice++ )
  {
    int y = (row * SCANLINES_PER_ROW * SCANLINES_YSCALE) + (slice * SCANLINES_YSCALE) ;

    // clear the scanline slice buffer...
    for ( byte b = 0 ; b < SCANLINE_WIDTH ; b++ ) buffer[b] = 0 ;

    // step through each polygon and render this scanline slice...
    byte *p_poly = &_drawing[0] ;
    byte polycount = *p_poly ;
    p_poly++;

    //Serial.print("polycount=" ) ;
    //Serial.println((int)polycount) ;

    for ( byte polyindex = 0 ; polyindex < polycount ; polyindex++ )
    {
      byte vertex_count = 0;
      byte fill_colour = 0;
      byte fill_pattern = 0b11111111 ; // default pixel pattern to fill with...
      // now convert the fill 'colour' into a pixel pattern...

      //      poly_read( p_poly, vertex_count, fill_colour, vertex, slope ) ; // fill the vertex and slope arrays from the flash data
      vertex_count = p_poly[0] ;
      fill_colour = p_poly[1] ;
      byte linewith = fill_colour >> 2 ;

      fill_pattern = get_pattern( row, fill_colour & 0b11 ) ;
      // now (finally) render a slice of this polygon into the buffer...
      // firstly retrieve intersections...
      byte intersect_count = 0;
      poly_scanline_intersections( y, p_poly, intersect_count, segments );

      // and then render the intersections...

      byte depth = 0 ;
      int start ;
      for ( byte i = 0 ; i < intersect_count ; i++ )
      {
        int crossing = segments[i] ;
        if (crossing >= 0) {
          if (depth == 0) { // this is the start of a segment.
            start = crossing ;
          }
          depth++ ;
        } else {
          depth-- ;
          if (depth == 0) { // this is the end of a segment.
            line( buffer, start, abs(crossing), fill_pattern ) ;
          }
        }
      }
      if (linewith != 0) {
        p_poly += 6 + ((vertex_count - 1) * 8) ;
      }
      else
      {
        p_poly += 2 + (vertex_count * 4) ;
      }
    }

    // now add the number of pixels set into the sum...
    for ( byte b = 0 ; b < SCANLINE_WIDTH ; b++ ) sum[b] += bits_set( buffer[b] ) ;
  } // do the next slice of 8...

  // now copy the sum into the buffer, dividing by the number of scanlines per row, to get a figure of 0-8 ...
  for (byte b = 0 ; b < SCANLINE_WIDTH ; b++ ) {
    buffer[b] = sum[b] / SCANLINES_PER_ROW ;
  }

  // and draw the buffer at the given row...
  output_buffer( buffer, row ) ;

}

//void drawpolygons( byte f_polys[] PROGMEM )
//{
//  for( byte row = 24 ; row > 0 ; row-- )
//  {
//    // render a row at a time, starting from the top of the screen...
//    draw_drawing_row( f_polys, row ) ;
//  }
//}

byte get_pattern( byte row, byte colour )
{
  byte pattern ;
  switch ( colour )
  {
    case 0 :
      pattern = 0b00000000 ;
      break ;
    case 1 :
      pattern = 0b11111111 ;
      break ;
    case 2 : // mid grey
      pattern = (row & 1) ? 0b10101010 : 0b01010101 ;
      break ;
    case 3 : // very light grey
      pattern = (row & 1) ? 0b10001000 : 0b00100010 ;
      break ;
    case 4 : // dark grey
      pattern = (row & 1) ? 0b01110111 : 0b11011101 ;
      break ;
    default: // black
      pattern = 0b11111111 ;
      break ;
  }
  return pattern ;
}

void output_buffer( byte buf[], int y )
{
  static const byte s_texture[] = { ' ', '.', '-', '=', '+', '*', '$', 'H', '#' };

  tm_move( y, 0 );

  for ( byte pos = 0 ; pos < SCANLINE_WIDTH ; pos++ )
  {
    byte n = buf[pos];//bits_set( buf[pos] );
    byte o = s_texture[ n ] ;
    tm_safe_write( o ) ;
  }
#ifdef SIMPLE
  Serial.println();
#endif
}

byte bits_set( byte a )
{
  byte mask = 0b10000000;
  byte c = 0;
  while (mask)
  {
    if (mask & a) c++ ;
    mask >>= 1;
  }
  return c ;
}

void line( byte buf[], int s, int e, byte pattern )
{
  static const byte s_startmask[8] = {0b11111111, 0b01111111, 0b00111111, 0b00011111, 0b00001111, 0b00000111, 0b00000011, 0b00000001 };
  static const byte s_endmask[8] = { 0b000000000, 0b10000000, 0b11000000, 0b11100000, 0b11110000, 0b11111000, 0b11111100, 0b11111110 };

  //  Serial.print("s = ");
  //  Serial.print(s);
  //  Serial.print(",e = ");
  //  Serial.println(e);
  //  return;

  // draw a horizontal line in a bit buffer between s and e bits.
  // s is included in the line. e is not.
  if (s < 0) s = 0;
  if (e > ((SCANLINE_WIDTH * 8) - 1)) e = (SCANLINE_WIDTH * 8) - 1;
  if (e <= 0) return ;
  if (s > ((SCANLINE_WIDTH * 8) - 1)) return ;

  // which bytes will be modified...
  byte pos_s = s >> 3 ;
  byte pos_e = e >> 3 ;

#define setpixels( screen, mask, pattern ) (((screen) & ~(mask))|((mask) & (pattern)))

  // which bits at the end will be modified...
  byte mask_s = s_startmask[ s & 0b111 ] ;
  byte mask_e = s_endmask[ e & 0b111 ] ;
  byte mask = mask_s ;
  if (pos_s == pos_e) {
    // special case: start and end pos are the same byte
    // put the start and end masks together.
    buf[ pos_s ] = setpixels( buf[pos_s], mask_s & mask_e, pattern) ;
    //    buf[pos_s] = buf[pos_s] | (mask_s & mask_e) ;
  }
  else {
    // start and end positions are different.
    byte pos = pos_s ;
    // render the start byte...
    buf[pos] = setpixels( buf[pos], mask_s, pattern ) ;
    //    buf[pos] = buf[pos] | mask_s ;
    pos++ ;
    // render any middle bytes...
    while (pos < pos_e) {
      buf[pos] = setpixels( buf[pos], 0b11111111, pattern ) ;
      pos++ ;
      //      buf[pos++] = 0b11111111 ;
    }
    // render the end byte...
    buf[pos] = setpixels( buf[pos], mask_e, pattern ) ;
    //    buf[pos] = buf[pos] | mask_e ;
  }
}

// tellymate helper functions
void tm_move( uint8_t row , uint8_t col )
{ // <ESC>Yrc
#ifndef SIMPLE
  Serial.print( CHAR_ESC "Y" ) ;
  Serial.print((char)(32 + row)) ;
  Serial.print((char)(32 + col)) ;
#endif
}

void tm_safe_write( byte t ) {
  if (t < 32) Serial.write( CHAR_DLE ) ;
  Serial.write( t ) ;
}

inline void tm_set_screen_fontbank( byte fontbank )
{
#ifndef SIMPLE
  if (fontbank != 0)
  {
    for ( byte row = 0 ; row < 25 ; row++ )
    {
      tm_move( row , 0 ) ; // move the cursor the required row (any column would do)
      Serial.print( CHAR_ESC );
      Serial.print( '_' );
      Serial.print( ((byte)('6' + GRAPHICS_FONTBANK))) ; // use the specified fontbank
    }
  }
#endif
}

void tm_cursor_show( bool show )
{ // <ESC>e or <ESC>f
#ifndef SIMPLE
  Serial.print( CHAR_ESC ) ;
  Serial.print( show ? 'e' : 'f' ) ;
#endif
}


void tm_cursor_restore( void )
{
#ifndef SIMPLE
  Serial.print( CHAR_ESC ) ;
  Serial.print( 'k' ) ;
#endif
}

void tm_clear_to_top( void )
{
#ifndef SIMPLE
  Serial.print( CHAR_ESC ) ;
  Serial.print( 'b' ) ;
#endif
}

void tm_clear_to_end( void )
{
#ifndef SIMPLE
  Serial.print( CHAR_ESC ) ;
  Serial.print( 'J' ) ;
#endif
}
