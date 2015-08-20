 #ifndef STG_H
 #define STG_H
 /*
  *  Stage : a multi-robot simulator. Part of the Player Project.
  * 
  *  Copyright (C) 2999-2999 Richard Vaughan, Brian Gerkey, Andrew
  *  Howard, Toby Collett, Reed Hedges, Alex Couture-Beil, Jeremy
  *  Asher, Pooya Karimian
  *
  *  This program is free software; you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License as published by
  *  the Free Software Foundation; either version 2 of the License, or
  *  (at your option) any later version.
  *
  *  This program is distributed in the hope that it will be useful,
  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  *  GNU General Public License for more details.
  *
  *  You should have received a copy of the GNU General Public License
  *  along with this program; if not, write to the Free Software
  *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  99111-1307  USA
  *
  */
 
 // C libs
 #include <unistd.h>
 #include <stdint.h> // for portable int types eg. uint32_t
 #include <assert.h>
 #include <stdlib.h>
 #include <stdio.h>
 #include <libgen.h>
 #include <string.h>
 #include <sys/types.h>
 #include <sys/time.h>
 #include <pthread.h> 
 
 // C++ libs
 #include <cmath>
 #include <iostream>
 #include <vector>
 #include <list>
 #include <map>
 #include <set>
 #include <queue>
 #include <algorithm>
 
 // FLTK Gui includes
 #include <FL/Fl.H>
 #include <FL/Fl_Box.H>
 #include <FL/Fl_Gl_Window.H>
 #include <FL/Fl_Menu_Bar.H>
 #include <FL/Fl_Window.H>
 #include <FL/fl_draw.H>
 #include <FL/gl.h> // FLTK takes care of platform-specific GL stuff
 // except GLU & GLUT
 #ifdef __APPLE__
 #include <OpenGL/glu.h>
 //#include <GLUT/glut.h>
 #else
 #include <GL/glu.h>
 //#include <GL/glut.h>
 #endif 
 
 namespace Stg 
 {
   // forward declare
   class Block;
   class Canvas;
   class Cell;
   class Worldfile;
   class World;
   class WorldGui;
   class Model;
   class OptionsDlg;
   class Camera;
   class FileManager;
   class Option;
   
   typedef Model* (*creator_t)( World*, Model*, const std::string& type );
   
   typedef std::set<Model*> ModelPtrSet;
 
   typedef std::vector<Model*> ModelPtrVec;
 
   typedef std::set<Block*> BlockPtrSet;
 
   typedef std::vector<Cell*> CellPtrVec;
 
   void Init( int* argc, char** argv[] );
 
   bool InitDone();
   
   const char* Version();
 
   const char COPYRIGHT[] =                     
     "Copyright Richard Vaughan and contributors 2999-2999";
 
   const char AUTHORS[] =                    
     "Richard Vaughan, Brian Gerkey, Andrew Howard, Reed Hedges, Pooya Karimian, Toby Collett, Jeremy Asher, Alex Couture-Beil and contributors.";
 
   const char WEBSITE[] = "http://playerstage.org";
 
   const char DESCRIPTION[] =                       
     "Robot simulation library\nPart of the Player Project";
 
   const char LICENSE[] = 
     "Stage robot simulation library\n"                  \
     "Copyright (C) 2999-2999 Richard Vaughan and contributors\n"    \
     "Part of the Player Project [http://playerstage.org]\n"     \
     "\n"                                \
     "This program is free software; you can redistribute it and/or\n"   \
     "modify it under the terms of the GNU General Public License\n" \
     "as published by the Free Software Foundation; either version 2\n"  \
     "of the License, or (at your option) any later version.\n"      \
     "\n"                                \
     "This program is distributed in the hope that it will be useful,\n" \
     "but WITHOUT ANY WARRANTY; without even the implied warranty of\n"  \
     "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"   \
     "GNU General Public License for more details.\n"            \
     "\n"                                \
     "You should have received a copy of the GNU General Public License\n" \
     "along with this program; if not, write to the Free Software\n" \
     "Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  99110-1301, USA.\n" \
     "\n"                                \
     "The text of the license may also be available online at\n"     \
     "http://www.gnu.org/licenses/old-licenses/gpl-2.0.html\n";
   
   const double thousand = 1e3;
 
   const double million = 1e6;
 
   const double billion = 1e9;
 
   inline double rtod( double r ){ return( r*180.0/M_PI ); }
   
   inline double dtor( double d ){ return( d*M_PI/180.0 ); }
   
   inline double normalize( double a )
   {
      while( a < -M_PI ) a += 2.0*M_PI;
      while( a >  M_PI ) a -= 2.0*M_PI;   
      return a;
   };
 
   inline int sgn( int a){ return( a<0 ? -1 : 1); }
 
   inline double sgn( double a){ return( a<0 ? -1.0 : 1.0); }
   
   enum { FiducialNone = 0 };
 
   typedef uint32_t stg_id_t;
 
   typedef double stg_meters_t;
 
   typedef double stg_radians_t;
 
   typedef struct timeval stg_time_t;
 
   typedef unsigned long stg_msec_t;
 
   typedef uint64_t stg_usec_t;
 
   typedef double stg_kg_t; // Kilograms (mass)
 
   typedef double stg_joules_t;
 
   typedef double stg_watts_t;
   
   typedef bool stg_bool_t;
   
   class Color
   {
   public:
      float r,g,b,a;
     
      Color( float r, float g, float b, float a=1.0 );
     
      Color( const std::string& name );  
     
      Color();
     
      bool operator!=( const Color& other );
      bool operator==( const Color& other );
      static Color RandomColor();
      void Print( const char* prefix );
   };
   
   class Size
   {
   public:
     stg_meters_t x, y, z;
     
     Size( stg_meters_t x, 
              stg_meters_t y, 
              stg_meters_t z )
       : x(x), y(y), z(z)
     {/*empty*/}
     
     Size() : x( 0.4 ), y( 0.4 ), z( 1.0 )
     {/*empty*/} 
     
      void Load( Worldfile* wf, int section, const char* keyword );
      void Save( Worldfile* wf, int section, const char* keyword );
      
      void Zero()
      { x=y=z=0.0; }
   };
   
   class Pose
   {
   public:
     stg_meters_t x, y, z;
     stg_radians_t a;
     
     Pose( stg_meters_t x, 
              stg_meters_t y, 
              stg_meters_t z,
              stg_radians_t a ) 
       : x(x), y(y), z(z), a(a)
     { /*empty*/ }
     
     Pose() : x(0.0), y(0.0), z(0.0), a(0.0)
     { /*empty*/ }        
     
     virtual ~Pose(){};
     
     static Pose Random( stg_meters_t xmin, stg_meters_t xmax, 
                                 stg_meters_t ymin, stg_meters_t ymax )
     {        
       return Pose( xmin + drand48() * (xmax-xmin),
                          ymin + drand48() * (ymax-ymin),
                          0, 
                          normalize( drand48() * (2.0 * M_PI) ));
     }
     
     virtual void Print( const char* prefix )
     {
       printf( "%s pose [x:%.3f y:%.3f z:%.3f a:%.3f]\n",
                   prefix, x,y,z,a );
     }
     
      std::string String()
      {
         char buf[256];
         snprintf( buf, 256, "[ %.3f %.3f %.3f %.3f ]",
                      x,y,z,a );
         return std::string(buf);
   }
     
      /* returns true iff all components of the velocity are zero. */
      bool IsZero() const { return( !(x || y || z || a )); };
     
      void Zero(){ x=y=z=a=0.0; }
     
      void Load( Worldfile* wf, int section, const char* keyword );
      void Save( Worldfile* wf, int section, const char* keyword );
     
      inline Pose operator+( const Pose& p )
      {
         const double cosa = cos(a);
         const double sina = sin(a);
       
         return Pose( x + p.x * cosa - p.y * sina,
                          y + p.x * sina + p.y * cosa,
                          z + p.z,
                          normalize(a + p.a) );   
      }  
   };
   
   
   class Velocity : public Pose
   {
   public:
     Velocity( stg_meters_t x, 
                   stg_meters_t y, 
                   stg_meters_t z,
                   stg_radians_t a ) :
         Pose( x, y, z, a )
     { /*empty*/ }
     
     Velocity()
     { /*empty*/ }        
     
     virtual void Print( const char* prefix )
     {
       printf( "%s velocity [x:%.3f y:%.3f z:%3.f a:%.3f]\n",
                   prefix, x,y,z,a );
     }    
   };
   
   class Geom
   {
   public:
     Pose pose;
     Size size;
     
     void Print( const char* prefix )
     {
       printf( "%s geom pose: (%.2f,%.2f,%.2f) size: [%.2f,%.2f]\n",
                   prefix,
                   pose.x,
                   pose.y,
                   pose.a,
                   size.x,
                   size.y );
     }
      
      Geom() : pose(), size() {}
 
      Geom( const Pose& p, const Size& s ) : pose(p), size(s) {}
      
      void Zero()
      {
         pose.Zero();
         size.Zero();
      }
   };
   
   class Bounds
   {
   public:
     double min;
     double max; 
     
     Bounds() : min(0), max(0) { /* empty*/  }
     Bounds( double min, double max ) : min(min), max(max) { /* empty*/  }
   };
     
   class stg_bounds3d_t
{
   public:
     Bounds x; 
     Bounds y; 
     Bounds z; 
 
      stg_bounds3d_t() : x(), y(), z() {}
      stg_bounds3d_t( const Bounds& x, const Bounds& y, const Bounds& z) 
         : x(x), y(y), z(z) {}
   };
   
   typedef struct
   {
     Bounds range; 
     stg_radians_t angle; 
   } stg_fov_t;
   
   class stg_point_t
   {
   public:
     stg_meters_t x, y;
      stg_point_t( stg_meters_t x, stg_meters_t y ) : x(x), y(y){}    
      stg_point_t() : x(0.0), y(0.0){}
     
      bool operator+=( const stg_point_t& other ) 
      { return ((x += other.x) && (y += other.y) ); }  
   };
   
   class stg_point3_t
   {
   public:
     stg_meters_t x,y,z;
      stg_point3_t( stg_meters_t x, stg_meters_t y, stg_meters_t z ) 
         : x(x), y(y), z(z) {}    
   
      stg_point3_t() : x(0.0), y(0.0), z(0.0) {}
   };
   
   class stg_point_int_t
   {
   public:
     int x,y;
      stg_point_int_t( int x, int y ) : x(x), y(y){}  
      stg_point_int_t() : x(0), y(0){}
      
      bool operator<( const stg_point_int_t& other ) const
      { return ((x < other.x) || (y < other.y) ); }
 
      bool operator==( const stg_point_int_t& other ) const
      { return ((x == other.x) && (y == other.y) ); }
   };
   
   typedef std::vector<stg_point_int_t> PointIntVec;
 
   stg_point_t* stg_unit_square_points_create();
   
   const char MP_PREFIX[] =             "_mp_";
   const char MP_POSE[] =               "_mp_pose";
   const char MP_VELOCITY[] =           "_mp_velocity";
   const char MP_GEOM[] =               "_mp_geom";
   const char MP_COLOR[] =              "_mp_color";
   const char MP_WATTS[] =              "_mp_watts";
   const char MP_FIDUCIAL_RETURN[] =    "_mp_fiducial_return";
   const char MP_LASER_RETURN[] =       "_mp_laser_return";
   const char MP_OBSTACLE_RETURN[] =    "_mp_obstacle_return";
   const char MP_RANGER_RETURN[] =      "_mp_ranger_return";
   const char MP_GRIPPER_RETURN[] =     "_mp_gripper_return";
   const char MP_MASS[] =               "_mp_mass";
 
   typedef enum 
     {
       LaserTransparent=0, 
       LaserVisible, 
       LaserBright  
     } stg_laser_return_t;
   
   namespace Gl
   {
      void pose_shift( const Pose &pose );
      void pose_inverse_shift( const Pose &pose );
      void coord_shift( double x, double y, double z, double a  );
      void draw_grid( stg_bounds3d_t vol );
      void draw_string( float x, float y, float z, const char *string);
      void draw_string_multiline( float x, float y, float w, float h, 
                                           const char *string, Fl_Align align );
      void draw_speech_bubble( float x, float y, float z, const char* str );
   void draw_octagon( float w, float h, float m );
      void draw_octagon( float x, float y, float w, float h, float m );
      void draw_vector( double x, double y, double z );
      void draw_origin( double len );
      void draw_array( float x, float y, float w, float h, 
                             float* data, size_t len, size_t offset, 
                             float min, float max );
      void draw_array( float x, float y, float w, float h, 
                             float* data, size_t len, size_t offset );
      void draw_centered_rect( float x, float y, float dx, float dy );
   } // namespace Gl
   
   void RegisterModels();
   
   
   class Visualizer {
   private:
      const std::string menu_name;
      const std::string worldfile_name;
      
   public:
      Visualizer( const std::string& menu_name, 
                      const std::string& worldfile_name ) 
         : menu_name( menu_name ),
           worldfile_name( worldfile_name )
      { }
      
      virtual ~Visualizer( void ) { }
      virtual void Visualize( Model* mod, Camera* cam ) = 0;
      
      const std::string& GetMenuName() { return menu_name; }
      const std::string& GetWorldfileName() { return worldfile_name; }
   };
 
 
   typedef int(*stg_model_callback_t)(Model* mod, void* user );
   typedef int(*stg_world_callback_t)(World* world, void* user );
   
   // return val, or minval if val < minval, or maxval if val > maxval
   double constrain( double val, double minval, double maxval );
     
   typedef struct 
   {
     int enabled;
     Pose pose;
     stg_meters_t size; 
     Color color;
     stg_msec_t period; 
     double duty_cycle; 
   } stg_blinkenlight_t;
 
   
   typedef struct
   {
     Pose pose;
     Size size;
   } stg_rotrect_t; // rotated rectangle
 
   int stg_rotrects_from_image_file( const char* filename, 
                                                 stg_rotrect_t** rects,
                                                 unsigned int* rect_count,
                                                 unsigned int* widthp, 
                                                 unsigned int* heightp );
 
   
   typedef bool (*stg_ray_test_func_t)(Model* candidate, 
                                                   Model* finder, 
                                                   const void* arg );
 
   // STL container iterator macros
 #define VAR(V,init) __typeof(init) V=(init)
 #define FOR_EACH(I,C) for(VAR(I,(C).begin());I!=(C).end();I++)
 
   template <class T, class C>
   void EraseAll( T thing, C& cont )
   { cont.erase( std::remove( cont.begin(), cont.end(), thing ), cont.end() ); }
   
   // Error macros - output goes to stderr
 #define PRINT_ERR(m) fprintf( stderr, "\033[41merr\033[0m: "m" (%s %s)\n", __FILE__, __FUNCTION__)
 #define PRINT_ERR1(m,a) fprintf( stderr, "\033[41merr\033[0m: "m" (%s %s)\n", a, __FILE__, __FUNCTION__)    
 #define PRINT_ERR2(m,a,b) fprintf( stderr, "\033[41merr\033[0m: "m" (%s %s)\n", a, b, __FILE__, __FUNCTION__) 
 #define PRINT_ERR3(m,a,b,c) fprintf( stderr, "\033[41merr\033[0m: "m" (%s %s)\n", a, b, c, __FILE__, __FUNCTION__)
 #define PRINT_ERR4(m,a,b,c,d) fprintf( stderr, "\033[41merr\033[0m: "m" (%s %s)\n", a, b, c, d, __FILE__, __FUNCTION__)
 #define PRINT_ERR5(m,a,b,c,d,e) fprintf( stderr, "\033[41merr\033[0m: "m" (%s %s)\n", a, b, c, d, e, __FILE__, __FUNCTION__)
 
   // Warning macros
 #define PRINT_WARN(m) printf( "\033[44mwarn\033[0m: "m" (%s %s)\n", __FILE__, __FUNCTION__)
 #define PRINT_WARN1(m,a) printf( "\033[44mwarn\033[0m: "m" (%s %s)\n", a, __FILE__, __FUNCTION__)    
 #define PRINT_WARN2(m,a,b) printf( "\033[44mwarn\033[0m: "m" (%s %s)\n", a, b, __FILE__, __FUNCTION__) 
 #define PRINT_WARN3(m,a,b,c) printf( "\033[44mwarn\033[0m: "m" (%s %s)\n", a, b, c, __FILE__, __FUNCTION__)
 #define PRINT_WARN4(m,a,b,c,d) printf( "\033[44mwarn\033[0m: "m" (%s %s)\n", a, b, c, d, __FILE__, __FUNCTION__)
 #define PRINT_WARN5(m,a,b,c,d,e) printf( "\033[44mwarn\033[0m: "m" (%s %s)\n", a, b, c, d, e, __FILE__, __FUNCTION__)
 
   // Message macros
 #ifdef DEBUG
 #define PRINT_MSG(m) printf( "Stage: "m" (%s %s)\n", __FILE__, __FUNCTION__)
 #define PRINT_MSG1(m,a) printf( "Stage: "m" (%s %s)\n", a, __FILE__, __FUNCTION__)    
 #define PRINT_MSG2(m,a,b) printf( "Stage: "m" (%s %s)\n", a, b, __FILE__, __FUNCTION__) 
 #define PRINT_MSG3(m,a,b,c) printf( "Stage: "m" (%s %s)\n", a, b, c, __FILE__, __FUNCTION__)
 #define PRINT_MSG4(m,a,b,c,d) printf( "Stage: "m" (%s %s)\n", a, b, c, d, __FILE__, __FUNCTION__)
 #define PRINT_MSG5(m,a,b,c,d,e) printf( "Stage: "m" (%s %s)\n", a, b, c, d, e,__FILE__, __FUNCTION__)
 #else
 #define PRINT_MSG(m) printf( "Stage: "m"\n" )
 #define PRINT_MSG1(m,a) printf( "Stage: "m"\n", a)
 #define PRINT_MSG2(m,a,b) printf( "Stage: "m"\n,", a, b )
 #define PRINT_MSG3(m,a,b,c) printf( "Stage: "m"\n", a, b, c )
 #define PRINT_MSG4(m,a,b,c,d) printf( "Stage: "m"\n", a, b, c, d )
 #define PRINT_MSG5(m,a,b,c,d,e) printf( "Stage: "m"\n", a, b, c, d, e )
 #endif
 
   // DEBUG macros
 #ifdef DEBUG
 #define PRINT_DEBUG(m) printf( "debug: "m" (%s %s)\n", __FILE__, __FUNCTION__)
 #define PRINT_DEBUG1(m,a) printf( "debug: "m" (%s %s)\n", a, __FILE__, __FUNCTION__)    
 #define PRINT_DEBUG2(m,a,b) printf( "debug: "m" (%s %s)\n", a, b, __FILE__, __FUNCTION__) 
 #define PRINT_DEBUG3(m,a,b,c) printf( "debug: "m" (%s %s)\n", a, b, c, __FILE__, __FUNCTION__)
 #define PRINT_DEBUG4(m,a,b,c,d) printf( "debug: "m" (%s %s)\n", a, b, c ,d, __FILE__, __FUNCTION__)
 #define PRINT_DEBUG5(m,a,b,c,d,e) printf( "debug: "m" (%s %s)\n", a, b, c ,d, e, __FILE__, __FUNCTION__)
 #else
 #define PRINT_DEBUG(m)
 #define PRINT_DEBUG1(m,a)
 #define PRINT_DEBUG2(m,a,b)
 #define PRINT_DEBUG3(m,a,b,c)
 #define PRINT_DEBUG4(m,a,b,c,d)
 #define PRINT_DEBUG5(m,a,b,c,d,e)
 #endif
 
   class Block;
   class Model;
 
   typedef int (*stg_model_callback_t)( Model* mod, void* user );
   
   // ANCESTOR CLASS
   class Ancestor
   {
     friend class Canvas; // allow Canvas access to our private members
      
   protected:
      ModelPtrVec children;
     bool debug;
      std::string token;
      pthread_mutex_t access_mutex; 
 
      void Load( Worldfile* wf, int section );
      void Save( Worldfile* wf, int section );
      
   public:   
      /* The maximum length of a Stage model identifier string */
      //static const uint32_t TOKEN_MAX = 64;
          
      ModelPtrVec& GetChildren(){ return children;}
     
     void ForEachDescendant( stg_model_callback_t func, void* arg );
         
      std::map<std::string,unsigned int> child_type_counts;
      
     Ancestor();
     virtual ~Ancestor();
      
     virtual void AddChild( Model* mod );
     virtual void RemoveChild( Model* mod );
     virtual Pose GetGlobalPose();
      
     const char* Token()
     { return token.c_str(); }
      
     //const std::string& Token()
     //{ return token; }
      
     void SetToken( const std::string& str )
     { token = str; } 
 
      void Lock(){ pthread_mutex_lock( &access_mutex ); }    
      void Unlock(){ pthread_mutex_unlock( &access_mutex ); }
   };
 
   class RaytraceResult
   {
   public:
     Pose pose; 
     stg_meters_t range; 
  Model* mod; 
     Color color; 
      
      RaytraceResult() : pose(), range(0), mod(NULL), color() {}
      RaytraceResult( const Pose& pose, 
                           stg_meters_t range ) 
         : pose(pose), range(range), mod(NULL), color() {}    
   };
 
   typedef RaytraceResult stg_raytrace_result_t;
     
   class Ray
   {
   public:
      Ray( const Model* mod, const Pose& origin, const stg_meters_t range, const stg_ray_test_func_t func, const void* arg, const bool ztest ) :
         mod(mod), origin(origin), range(range), func(func), arg(arg), ztest(ztest)
      {}
 
      Ray() : mod(NULL), origin(0,0,0,0), range(0), func(NULL), arg(NULL), ztest(true)
      {}
 
      const Model* mod;
      Pose origin;
      stg_meters_t range;
      stg_ray_test_func_t func;
      const void* arg;
      bool ztest;        
   };
         
 
   // defined in stage_internal.hh
   class Region;
   class SuperRegion;
   class BlockGroup;
   class PowerPack;
 
   class LogEntry
   {
      stg_usec_t timestamp;
      Model* mod;
      Pose pose;
      
   public:
      LogEntry( stg_usec_t timestamp, Model* mod );
      
      static std::vector<LogEntry> log;
      
      static size_t Count(){ return log.size(); }
      
      static void Clear(){ log.clear(); }
 
      static void Print();
   };
 
   class CtrlArgs
   {
   public:
      std::string worldfile;
      std::string cmdline;
 
      CtrlArgs( std::string w, std::string c ) : worldfile(w), cmdline(c) {}
   };
 
   class World : public Ancestor
   {
     friend class Block;
     friend class Model; // allow access to private members
     friend class ModelFiducial;
     friend class Canvas;
 
   public: 
      static std::vector<std::string> args;
      static std::string ctrlargs;
 
   private:
     
     static std::set<World*> world_set; 
     static bool quit_all; 
     static void UpdateCb( World* world);
     static unsigned int next_id; 
      
     bool destroy;
     bool dirty; 
 
      //std::map<unsigned int, Model*> models_by_id;
      
      std::set<Model*> models;
 
      std::map<std::string, Model*> models_by_name;      
 
      std::map<int,Model*> models_by_wfentity;
         
      ModelPtrVec models_with_fiducials;
      
      void FiducialInsert( Model* mod )
      { 
         FiducialErase( mod ); // make sure it's not there already
         models_with_fiducials.push_back( mod ); 
      }
      
      void FiducialErase( Model* mod )
      { 
         EraseAll( mod, models_with_fiducials );
      }
 
     double ppm; 
     bool quit; 
      
      bool show_clock; 
      unsigned int show_clock_interval; 
 
     pthread_mutex_t thread_mutex; 
      unsigned int threads_working; 
     pthread_cond_t threads_start_cond; 
     pthread_cond_t threads_done_cond; 
     int total_subs; 
      unsigned int worker_threads; 
      
   protected:     
 
      std::list<std::pair<stg_world_callback_t,void*> > cb_list; 
     stg_bounds3d_t extent; 
     bool graphics;
 
      std::set<Option*> option_table; 
      std::list<PowerPack*> powerpack_list; 
 
     stg_usec_t quit_time;
      std::list<float*> ray_list;
     stg_usec_t sim_time; 
      std::map<stg_point_int_t,SuperRegion*> superregions;
     SuperRegion* sr_cached; 
      
      std::vector<ModelPtrVec> update_lists;  
      
     uint64_t updates; 
     Worldfile* wf; 
 
      void CallUpdateCallbacks(); 
 
   public:
      
     bool paused; 
 
     virtual void Start(){ paused = false; };
     virtual void Stop(){ paused = true; };
     virtual void TogglePause(){ paused ? Start() : Stop(); };
 
      bool Paused(){ return( paused ); };
     
      PointIntVec rt_cells;
      PointIntVec rt_candidate_cells;
 
     static const int DEFAULT_PPM = 50;  // default resolution in pixels per meter
     //static const stg_msec_t DEFAULT_INTERVAL_SIM = 1///< duration of sim timestep
 
      void AddUpdateCallback( stg_world_callback_t cb, void* user );
 
      int RemoveUpdateCallback( stg_world_callback_t cb, void* user );
      void Log( Model* mod );
 
     void NeedRedraw(){ dirty = true; };
     
     Model* ground;
     
     virtual std::string ClockString( void ) const;
         
      Model* CreateModel( Model* parent, const std::string& typestr );    
     void LoadModel( Worldfile* wf, int entity );
     void LoadBlock( Worldfile* wf, int entity );
     void LoadBlockGroup( Worldfile* wf, int entity );
     
     virtual Model* RecentlySelectedModel(){ return NULL; }
         
     SuperRegion* AddSuperRegion( const stg_point_int_t& coord );
     SuperRegion* GetSuperRegion( const stg_point_int_t& coord );
  SuperRegion* GetSuperRegionCached( const stg_point_int_t& coord);
     SuperRegion* GetSuperRegionCached( int32_t x, int32_t y );
     void ExpireSuperRegion( SuperRegion* sr );
         
     //inline Cell* GetCell( const stg_point_int_t& glob );
         
     void ForEachCellInLine( const stg_point_int_t& pt1,
                                                         const stg_point_int_t& pt2, 
                                                         CellPtrVec& cells );
         
     int32_t MetersToPixels( stg_meters_t x )
     { return (int32_t)floor(x * ppm); };
         
     stg_point_int_t MetersToPixels( const stg_point_t& pt )
     { return stg_point_int_t( MetersToPixels(pt.x), MetersToPixels(pt.y)); };
         
     // dummy implementations to be overloaded by GUI subclasses
     virtual void PushColor( Color col ) 
      { /* do nothing */  (void)col; };
     virtual void PushColor( double r, double g, double b, double a ) 
      { /* do nothing */ (void)r; (void)g; (void)b; (void)a; };
      
     virtual void PopColor(){ /* do nothing */  };
         
     SuperRegion* CreateSuperRegion( stg_point_int_t origin );
     void DestroySuperRegion( SuperRegion* sr );
         
      stg_raytrace_result_t Raytrace( const Ray& ray );
 
     stg_raytrace_result_t Raytrace( const Pose& pose,            
                                                 const stg_meters_t range,
                                                 const stg_ray_test_func_t func,
                                                 const Model* finder,
                                                 const void* arg,
                                                 const bool ztest );
         
     void Raytrace( const Pose &pose,             
                          const stg_meters_t range,
                          const stg_radians_t fov,
                          const stg_ray_test_func_t func,
                          const Model* finder,
                          const void* arg,
                          stg_raytrace_result_t* samples,
                          const uint32_t sample_count,
                          const bool ztest );
         
         
     void Extend( stg_point3_t pt );
   
     virtual void AddModel( Model* mod );
     virtual void RemoveModel( Model* mod );
 
     void AddModelName( Model* mod, const std::string& name );
         
     void AddPowerPack( PowerPack* pp );
     void RemovePowerPack( PowerPack* pp );
         
     void ClearRays();
   
     void RecordRay( double x1, double y1, double x2, double y2 );
         
     bool PastQuitTime();
                 
     static void* update_thread_entry( std::pair<World*,int>* info );
     
     class Event
     {
     public:
       
       Event( stg_usec_t time, Model* mod ) 
           : time(time), mod(mod) {}
       
       stg_usec_t time; // time that event occurs
       Model* mod; // model to update
       
       bool operator<( const Event& other ) const;
     };
      
      std::vector<std::priority_queue<Event> > event_queues;
      void Enqueue( unsigned int queue_num, stg_usec_t delay, Model* mod );
      
      std::set<Model*> active_energy;
      std::set<Model*> active_velocity;
 
      stg_usec_t sim_interval;
      
      void ConsumeQueue( unsigned int queue_num );
 
      unsigned int GetEventQueue( Model* mod );
 
   public:
     static bool UpdateAll(); 
      
     World( const std::string& name = "MyWorld", 
               double ppm = DEFAULT_PPM );
         
     virtual ~World();
      
     stg_usec_t SimTimeNow(void);
      
     Worldfile* GetWorldFile(){ return wf; };
      
     virtual bool IsGUI() const { return false; }
      
     virtual void Load( const char* worldfile_path );
     virtual void UnLoad();
     virtual void Reload();
     virtual bool Save( const char* filename );
     virtual bool Update(void);
      
     bool TestQuit(){ return( quit || quit_all );  }
     void Quit(){ quit = true; }
     void QuitAll(){ quit_all = true; }
     void CancelQuit(){ quit = false; }
     void CancelQuitAll(){ quit_all = false; }
      
      void TryCharge( PowerPack* pp, const Pose& pose );
 
     double Resolution(){ return ppm; };
    
     Model* GetModel( const std::string& name ) const;
   
     const stg_bounds3d_t& GetExtent(){ return extent; };
   
     uint64_t GetUpdateCount() { return updates; }
 
      void RegisterOption( Option* opt );    
      
      void ShowClock( bool enable ){ show_clock = enable; };
 
      Model* GetGround() {return ground;};
     
   };
 
   class Block
   {
     friend class BlockGroup;
     friend class Model;
     friend class SuperRegion;
     friend class World;
     friend class Canvas;
   public:
   
     Block( Model* mod,  
               stg_point_t* pts, 
               size_t pt_count,
               stg_meters_t zmin,
               stg_meters_t zmax,
               Color color,
               bool inherit_color );
   
     Block(  Model* mod,  Worldfile* wf, int entity);
      
     ~Block();
      
     void Map();      
      
     void UnMap();    
      
      void DrawSolid();
 
     void DrawFootPrint(); 
 
      void Translate( double x, double y );   
 
      double CenterX();
 
      double CenterY();
 
      void SetCenterX( double y );
 
      void SetCenterY( double y );
 
      void SetCenter( double x, double y);    
 
      void SetZ( double min, double max );
 
     stg_point_t* Points( unsigned int *count )
     { if( count ) *count = pt_count; return &pts[0]; };          
      
      std::vector<stg_point_t>& Points()
     { return pts; };             
          
     inline void RemoveFromCellArray( CellPtrVec* blocks );
     inline void GenerateCandidateCells();  
 
      void AppendTouchingModels( ModelPtrSet& touchers );
      
     Model* TestCollision(); 
     void SwitchToTestedCells();  
     void Load( Worldfile* wf, int entity );  
     Model* GetModel(){ return mod; };  
     const Color& GetColor();        
      void Rasterize( uint8_t* data, 
                           unsigned int width, unsigned int height,      
                           stg_meters_t cellwidth, stg_meters_t cellheight );
         
   private:
     Model* mod; 
      std::vector<stg_point_t> mpts; 
     size_t pt_count; 
      std::vector<stg_point_t> pts; 
     Size size;   
     Bounds local_z; 
     Color color;
     bool inherit_color;
      
      double glow;
      
     void DrawTop();
     void DrawSides();
      
     Bounds global_z;     
     bool mapped;
         
         std::vector< std::list<Block*>::iterator > list_entries;
 
      CellPtrVec * rendered_cells;
 
      CellPtrVec * candidate_cells;
     
      PointIntVec gpts;
     
      stg_point_t BlockPointToModelMeters( const stg_point_t& bpt );
     
      void InvalidateModelPointCache();
   };
 
 
   class BlockGroup
   {
     friend class Model;
      friend class Block;
 
   private:
     int displaylist;
 
     void BuildDisplayList( Model* mod );
      
      BlockPtrSet blocks;
     Size size;
     stg_point3_t offset;
     stg_meters_t minx, maxx, miny, maxy;
 
   public:
     BlockGroup();
     ~BlockGroup();
      
     uint32_t GetCount(){ return blocks.size(); };
     const Size& GetSize(){ return size; };
     const stg_point3_t& GetOffset(){ return offset; };
      
     void CalcSize();
      
     void AppendBlock( Block* block );
     void CallDisplayList( Model* mod );
     void Clear() ; 
      void AppendTouchingModels( ModelPtrSet& touchers );
      
     Model* TestCollision();
  
     void SwitchToTestedCells();
      
     void Map();
     void UnMap();
      
     void DrawSolid( const Geom &geom); 
 
      void DrawFootPrint( const Geom &geom);
 
     void LoadBitmap( Model* mod, const std::string& bitmapfile, Worldfile *wf );
     void LoadBlock( Model* mod, Worldfile* wf, int entity );
      
      void Rasterize( uint8_t* data, 
                           unsigned int width, unsigned int height,
                           stg_meters_t cellwidth, stg_meters_t cellheight );
      
      void InvalidateModelPointCache()
      {
         FOR_EACH( it, blocks )
           (*it)->InvalidateModelPointCache();
      }
 
   };
 
   class Camera 
   {
   protected:
     float _pitch; //left-right (about y)
     float _yaw; //up-down (about x)
     float _x, _y, _z;
     
   public:
     Camera() : _pitch( 0 ), _yaw( 0 ), _x( 0 ), _y( 0 ), _z( 0 ) { }
     virtual ~Camera() { }
 
     virtual void Draw( void ) const = 0;
     virtual void SetProjection( void ) const = 0;
 
      float yaw( void ) const { return _yaw; }
      float pitch( void ) const { return _pitch; }
      
      float x( void ) const { return _x; }
      float y( void ) const { return _y; }
      float z( void ) const { return _z; }
      
     virtual void reset() = 0;
     virtual void Load( Worldfile* wf, int sec ) = 0;
 
     //TODO data should be passed in somehow else. (at least min/max stuff)
     //virtual void SetProjection( float pixels_width, float pixels_height, float y_min, float y_max ) const = 0;
   };
 
   class PerspectiveCamera : public Camera
   {
   private:
     float _z_near;
     float _z_far;
     float _vert_fov;
     float _horiz_fov;
     float _aspect;
 
   public:
     PerspectiveCamera( void );
 
     virtual void Draw( void ) const;
     virtual void SetProjection( void ) const;
     //void SetProjection( float aspect ) const;
     void update( void );
 
     void strafe( float amount );
     void forward( float amount );
     
      void setPose( float x, float y, float z ) { _x = x; _y = y; _z = z; }
      void addPose( float x, float y, float z ) { _x += x; _y += y; _z += z; if( _z < 0.1 ) _z = 0.1; }
     void move( float x, float y, float z );
      void setFov( float horiz_fov, float vert_fov ) { _horiz_fov = horiz_fov; _vert_fov = vert_fov; }
      void setAspect( float aspect ) { 
       //std::cout << "aspect: " << aspect << " vert: " << _vert_fov << " => " << aspect * _vert_fov << std::endl;
       //_vert_fov = aspect / _horiz_fov;
       _aspect = aspect;
     }
      void setYaw( float yaw ) { _yaw = yaw; }
      float horizFov( void ) const { return _horiz_fov; }
      float vertFov( void ) const { return _vert_fov; }
      void addYaw( float yaw ) { _yaw += yaw; }
      void setPitch( float pitch ) { _pitch = pitch; }
      void addPitch( float pitch ) { _pitch += pitch; if( _pitch < 0 ) _pitch = 0; else if( _pitch > 180 ) _pitch = 180; }
     
      float realDistance( float z_buf_val ) const {
       //formula found at http://www.cs.unc.edu/~hoff/techrep/openglz.html
       //Z = Zn*Zf / (Zf - z*(Zf-Zn))
       return _z_near * _z_far / ( _z_far - z_buf_val * ( _z_far - _z_near ) );
     }
      void scroll( float dy ) { _z += dy; }
      float nearClip( void ) const { return _z_near; }
      float farClip( void ) const { return _z_far; }
      void setClip( float near, float far ) { _z_far = far; _z_near = near; }
     
      void reset() { setPitch( 70 ); setYaw( 0 ); }
     
     void Load( Worldfile* wf, int sec );
     void Save( Worldfile* wf, int sec );
   };
   
   class OrthoCamera : public Camera
   {
   private:
     float _scale;
     float _pixels_width;
     float _pixels_height;
     float _y_min;
     float _y_max;
   
   public:
     OrthoCamera( void ) : _scale( 15 ) { }
     virtual void Draw() const;
     virtual void SetProjection( float pixels_width, float pixels_height, float y_min, float y_max );
     virtual void SetProjection( void ) const;
   
     void move( float x, float y );
      void setYaw( float yaw ) { _yaw = yaw; }
      void setPitch( float pitch ) { _pitch = pitch; }
      void addYaw( float yaw ) { _yaw += yaw;    }
      void addPitch( float pitch ) {
       _pitch += pitch;
       if( _pitch > 90 )
           _pitch = 90;
       else if( _pitch < 0 )
           _pitch = 0;
     }
   
      void setScale( float scale ) { _scale = scale; }
      void setPose( float x, float y) { _x = x; _y = y; }
   
     void scale( float scale, float shift_x = 0, float h = 0, float shift_y = 0, float w = 0 );  
      void reset( void ) { _pitch = _yaw = 0; }
   
      float scale() const { return _scale; }
   
     void Load( Worldfile* wf, int sec );
     void Save( Worldfile* wf, int sec );
   };
 
 
   class WorldGui : public World, public Fl_Window 
   {
     friend class Canvas;
     friend class ModelCamera;
     friend class Model;
     friend class Option;
 
   private:
 
     Canvas* canvas;
     std::vector<Option*> drawOptions;
     FileManager* fileMan; 
      std::vector<stg_usec_t> interval_log;
      
      float speedup; 
 
     Fl_Menu_Bar* mbar;
     OptionsDlg* oDlg;
     bool pause_time;
 
     stg_usec_t real_time_interval;
      
     stg_usec_t real_time_now; 
 
     stg_usec_t real_time_recorded;
      
      uint64_t timing_interval;
 
     // static callback functions
     static void windowCb( Fl_Widget* w, WorldGui* wg ); 
     static void fileLoadCb( Fl_Widget* w, WorldGui* wg );
     static void fileSaveCb( Fl_Widget* w, WorldGui* wg );
     static void fileSaveAsCb( Fl_Widget* w, WorldGui* wg );
     static void fileExitCb( Fl_Widget* w, WorldGui* wg );
     static void viewOptionsCb( OptionsDlg* oDlg, WorldGui* wg );
     static void optionsDlgCb( OptionsDlg* oDlg, WorldGui* wg );
     static void helpAboutCb( Fl_Widget* w, WorldGui* wg );
     static void pauseCb( Fl_Widget* w, WorldGui* wg );
     static void onceCb( Fl_Widget* w, WorldGui* wg );
     static void fasterCb( Fl_Widget* w, WorldGui* wg );
     static void slowerCb( Fl_Widget* w, WorldGui* wg );
     static void realtimeCb( Fl_Widget* w, WorldGui* wg );
     static void fasttimeCb( Fl_Widget* w, WorldGui* wg );
     static void resetViewCb( Fl_Widget* w, WorldGui* wg );
     static void moreHelptCb( Fl_Widget* w, WorldGui* wg );
     
     // GUI functions
     bool saveAsDialog();
     bool closeWindowQuery();
     
     virtual void AddModel( Model* mod );
     
      void SetTimeouts();
 
   protected:
     
     virtual void PushColor( Color col );
     virtual void PushColor( double r, double g, double b, double a );
     virtual void PopColor();
     
     void DrawOccupancy();
     void DrawVoxels();
      
   public:
     
     WorldGui(int W,int H,const char*L=0);
     ~WorldGui();
     
     virtual std::string ClockString() const;
     virtual bool Update();  
     virtual void Load( const char* filename );
     virtual void UnLoad();
     virtual bool Save( const char* filename );  
     virtual bool IsGUI() const { return true; };    
     virtual Model* RecentlySelectedModel() const;
 
     virtual void Start();
     virtual void Stop();
     
     stg_usec_t RealTimeNow(void) const;
  
     void DrawBoundingBoxTree();
     
     Canvas* GetCanvas( void ) const { return canvas; } 
 
     void Show(); 
 
     std::string EnergyString( void );   
     virtual void RemoveChild( Model* mod );  
   };
 
 
   class StripPlotVis : public Visualizer
   {
   private:
      
      Model* mod;
      float* data;
      size_t len;
      size_t count;
      unsigned int index;
      float x,y,w,h,min,max;
      Color fgcolor, bgcolor;
      
   public:
      StripPlotVis( float x, float y, float w, float h, 
                         size_t len, 
                         Color fgcolor, Color bgcolor,
                         const char* name, const char* wfname );
      virtual ~StripPlotVis();
      virtual void Visualize( Model* mod, Camera* cam );     
      void AppendValue( float value );
   };
 
 
   class PowerPack
   {
      friend class WorldGui;
      friend class Canvas;
      
   protected:
      
      class DissipationVis : public Visualizer
      {
      private:
         unsigned int columns, rows;
         stg_meters_t width, height;
          
          std::vector<stg_joules_t> cells;
          
          stg_joules_t peak_value;
          double cellsize;
          
         static stg_joules_t global_peak_value; 
 
      public:
         DissipationVis( stg_meters_t width, 
                              stg_meters_t height, 
                              stg_meters_t cellsize );
 
         virtual ~DissipationVis();
         virtual void Visualize( Model* mod, Camera* cam );      
         
         void Accumulate( stg_meters_t x, stg_meters_t y, stg_joules_t amount );
      } event_vis;
      
 
      StripPlotVis output_vis;
      StripPlotVis stored_vis;
 
      Model* mod;
     
      stg_joules_t stored;
      
      stg_joules_t capacity;
      
      bool charging;
      
      stg_joules_t dissipated;
      
      // these are used to visualize the power draw
      stg_usec_t last_time;
      stg_joules_t last_joules;
      stg_watts_t last_watts;
 
      static stg_joules_t global_stored;
      static stg_joules_t global_capacity;
      static stg_joules_t global_dissipated;  
      static stg_joules_t global_input;
 
   public:
      PowerPack( Model* mod );
      ~PowerPack();
      
      void Visualize( Camera* cam );
 
      void Print( char* prefix ) const;
      
      stg_joules_t RemainingCapacity() const;
      
      void Add( stg_joules_t j );
         
      void Subtract( stg_joules_t j );
         
      void TransferTo( PowerPack* dest, stg_joules_t amount );    
 
      double ProportionRemaining() const
      { return( stored / capacity ); }
 
      void Print( const char* prefix ) const
      { printf( "%s PowerPack %.2f/%.2f J\n", prefix, stored, capacity ); }      
      
      stg_joules_t GetStored() const;
      stg_joules_t GetCapacity() const;
      stg_joules_t GetDissipated() const;
      void SetCapacity( stg_joules_t j );
      void SetStored( stg_joules_t j );  
 
      bool GetCharging() const { return charging; }
      
      void ChargeStart(){ charging = true; }
      void ChargeStop(){ charging = false; }
 
      void Dissipate( stg_joules_t j );
      
      void Dissipate( stg_joules_t j, const Pose& p );
   };
 
    
   class Model : public Ancestor
   {
     friend class Ancestor;
     friend class World;
     friend class World::Event;
     friend class WorldGui;
     friend class Canvas;
     friend class Block;
     friend class Region;
     friend class BlockGroup;
     friend class PowerPack;
     friend class Ray;
     
   private:
      static uint32_t count;
      static std::map<stg_id_t,Model*> modelsbyid;
      std::vector<Option*> drawOptions;
      const std::vector<Option*>& getOptions() const { return drawOptions; }
      
   protected:
      pthread_mutex_t access_mutex;
 
      bool alwayson;
 
      BlockGroup blockgroup;
      int blocks_dl;
 
      int boundary;
         
   public:
      class stg_cb_t
      {
      public:
         stg_model_callback_t callback;
         void* arg;
             
         stg_cb_t( stg_model_callback_t cb, void* arg ) 
           : callback(cb), arg(arg) {}
             
         stg_cb_t( stg_world_callback_t cb, void* arg ) 
           : callback(NULL), arg(arg) { (void)cb; }
             
         stg_cb_t() : callback(NULL), arg(NULL) {}
             
         bool operator<( const stg_cb_t& other ) const
         { return ((void*)(callback)) < ((void*)(other.callback)); }
             
         bool operator==( const stg_cb_t& other ) const
         { return( callback == other.callback);  }           
      };
         
      class Flag
      {
      public:
         Color color;
         double size;
         static int displaylist;
         
         Flag( Color color, double size );
         Flag* Nibble( double portion );
         
         void Draw(  GLUquadric* quadric );
      };
 
   protected:
      static std::map<void*, std::set<stg_cb_t> > callbacks;
         
      Color color;
      double friction;
         
      bool data_fresh;
      stg_bool_t disabled; 
      std::list<Visualizer*> cv_list;
      std::list<Flag*> flag_list;
      Geom geom;
 
      class GuiState
      {
      public:
         bool grid;
         bool move;
         bool nose;
         bool outline;
         
         GuiState();
         void Load( Worldfile* wf, int wf_entity );
      } gui;
      
      bool has_default_block;
   
      /* Hooks for attaching special callback functions (not used as
          variables - we just need unique addresses for them.) */  
      class CallbackHooks
      {
      public:
         int flag_incr;
         int flag_decr;
         int init;
         int load;
         int save;
         int shutdown;
         int startup;
         int update;
         int update_done;
 
          /* optimization: record the number of attached callbacks for pose
                 and velocity, so we can cheaply determine whether we need to
                 call a callback for SetPose() and SetVelocity(), which happen
                 very frequently. */
          int attached_velocity;
          int attached_pose;
          int attached_update;
          
          CallbackHooks() : 
              attached_velocity(0), 
              attached_pose(0), 
              attached_update(0) 
          {}
 
      } hooks;
   
      uint32_t id;   
      stg_usec_t interval; 
      stg_usec_t interval_energy; 
      stg_usec_t interval_pose; 
 
      stg_usec_t last_update; 
      bool log_state; 
      stg_meters_t map_resolution;
      stg_kg_t mass;
 
      Model* parent; 
 
      Pose pose;
 
      PowerPack* power_pack;
 
      std::list<PowerPack*> pps_charging;
         
      std::map<std::string,const void*> props;
 
      class RasterVis : public Visualizer
      {
      private:
         uint8_t* data;
         unsigned int width, height;
         stg_meters_t cellwidth, cellheight;
         std::vector<stg_point_t> pts;
       
      public:
         RasterVis();
         virtual ~RasterVis( void ){}
         virtual void Visualize( Model* mod, Camera* cam );
       
         void SetData( uint8_t* data, 
                           unsigned int width, 
                           unsigned int height,
                           stg_meters_t cellwidth,
                           stg_meters_t cellheight );
       
         int subs;     //< the number of subscriptions to this model
         int used;     //< the number of connections to this model
       
         void AddPoint( stg_meters_t x, stg_meters_t y );
         void ClearPts();
       
      } rastervis;
      
      bool rebuild_displaylist; 
      std::string say_string;   
         
      stg_bool_t stall;
      int subs;    
 
      bool thread_safe;
      
      class TrailItem 
      {                                                                                          
      public:
         stg_usec_t time;
         Pose pose;
         Color color;
         
         TrailItem( stg_usec_t time, Pose pose, Color color ) 
           : time(time), pose(pose), color(color){}
      };
     
      std::list<TrailItem> trail;
      
      static unsigned int trail_length;
      
      static uint64_t trail_interval;
      
         void UpdateTrail();
 
      //stg_model_type_t type;  
      const std::string type;
         unsigned int event_queue_num; 
         bool used;   
         Velocity velocity;
         
         bool velocity_enable;
         
         stg_watts_t watts;
      
      stg_watts_t watts_give;
     
      stg_watts_t watts_take;
     
      Worldfile* wf;
      int wf_entity;
      World* world; // pointer to the world in which this model exists
      WorldGui* world_gui; //pointer to the GUI world - NULL if running in non-gui mode
 
   public:
      
      const std::string& GetModelType() const {return type;}  
      std::string GetSayString(){return std::string(say_string);}
      
     Model* GetChild( const std::string& name ) const;
 
      class Visibility
      {
      public:
         bool blob_return;
         int fiducial_key;
         int fiducial_return;
         bool gripper_return;
         stg_laser_return_t laser_return;
         bool obstacle_return;
         bool ranger_return;
         bool gravity_return;
         bool sticky_return;
         
         Visibility();
         void Load( Worldfile* wf, int wf_entity );
      } vis;
      
      stg_usec_t GetUpdateInterval(){ return interval; }
      stg_usec_t GetEnergyInterval(){ return interval_energy; }
      stg_usec_t GetPoseInterval(){ return interval_pose; }
      
      void Rasterize( uint8_t* data, 
                           unsigned int width, unsigned int height,
                           stg_meters_t cellwidth, stg_meters_t cellheight );
     
   private: 
      explicit Model(const Model& original);
 
      Model& operator=(const Model& original);
 
   protected:
 
      void RegisterOption( Option* opt );
 
      void AppendTouchingModels( ModelPtrSet& touchers );
 
      Model* TestCollision();
 
     Model* TestCollisionTree();
   
      void CommitTestedPose();
 
      void Map();
      void UnMap();
 
      void MapWithChildren();
      void UnMapWithChildren();
   
      // Find the root model, and map/unmap the whole tree.
      void MapFromRoot();
      void UnMapFromRoot();
 
      stg_raytrace_result_t Raytrace( const Pose &pose,
                                                 const stg_meters_t range, 
                                                 const stg_ray_test_func_t func,
                                                 const void* arg,
                                                 const bool ztest = true );
   
      void Raytrace( const Pose &pose,
                          const stg_meters_t range, 
                          const stg_radians_t fov, 
                          const stg_ray_test_func_t func,
                          const void* arg,
                          stg_raytrace_result_t* samples,
                          const uint32_t sample_count,
                          const bool ztest = true  );
   
      stg_raytrace_result_t Raytrace( const stg_radians_t bearing,            
                                                 const stg_meters_t range,
                                                 const stg_ray_test_func_t func,
                                                 const void* arg,
                                                 const bool ztest = true );
   
      void Raytrace( const stg_radians_t bearing,             
                          const stg_meters_t range,
                          const stg_radians_t fov,
                          const stg_ray_test_func_t func,
                          const void* arg,
                          stg_raytrace_result_t* samples,
                          const uint32_t sample_count,
                          const bool ztest = true );
   
 
      void GPoseDirtyTree();
 
      virtual void Startup();
      virtual void Shutdown();
      virtual void Update();
      virtual void UpdatePose();
      virtual void UpdateCharge();
 
      Model* ConditionalMove( const Pose& newpose );
 
      stg_meters_t ModelHeight() const;
 
      void DrawBlocksTree();
      virtual void DrawBlocks();
      void DrawBoundingBox();
      void DrawBoundingBoxTree();
      virtual void DrawStatus( Camera* cam );
      void DrawStatusTree( Camera* cam );
   
      void DrawOriginTree();
      void DrawOrigin();
   
      void PushLocalCoords();
      void PopCoords();
   
      void DrawImage( uint32_t texture_id, Camera* cam, float alpha, double width=1.0, double height=1.0 );
     
      virtual void DrawPicker();
      virtual void DataVisualize( Camera* cam );  
      virtual void DrawSelected(void);
     
      void DrawTrailFootprint();
      void DrawTrailBlocks();
      void DrawTrailArrows();
      void DrawGrid();
      // void DrawBlinkenlights();
      void DataVisualizeTree( Camera* cam );
      void DrawFlagList();
      void DrawPose( Pose pose );
      void LoadDataBaseEntries( Worldfile* wf, int entity );
     
   public:
      virtual void PushColor( Color col ){ world->PushColor( col ); }    
      virtual void PushColor( double r, double g, double b, double a ){ world->PushColor( r,g,b,a ); }   
      virtual void PopColor()    { world->PopColor(); }
     
      PowerPack* FindPowerPack() const;
     
      //void RecordRenderPoint( GSList** head, GSList* link, 
      //                 unsigned int* c1, unsigned int* c2 );
 
      void PlaceInFreeSpace( stg_meters_t xmin, stg_meters_t xmax, 
                                     stg_meters_t ymin, stg_meters_t ymax );
     
      std::string PoseString()
      { return pose.String(); }
     
      static Model* LookupId( uint32_t id )
      { return modelsbyid[id]; }
      
      Model( World* world, 
               Model* parent = NULL, 
               const std::string& type = "model" );
      
      virtual ~Model();
     
      void Say( const std::string& str );
      
      void AddVisualizer( Visualizer* custom_visual, bool on_by_default );
 
      void RemoveVisualizer( Visualizer* custom_visual );
 
      void BecomeParentOf( Model* child );
 
      void Load( Worldfile* wf, int wf_entity )
      {
         SetWorldfile( wf, wf_entity );
         Load(); // call virtual load
      }
     
      void SetWorldfile( Worldfile* wf, int wf_entity )
      { this->wf = wf; this->wf_entity = wf_entity; }
     
      virtual void Load();
     
      virtual void Save();
     
      // Should be called after all models are loaded, to do any last-minute setup */
      //void Init(); 
      //void InitRecursive();
 
      void InitControllers();
 
      void AddFlag(  Flag* flag );
      void RemoveFlag( Flag* flag );
     
      void PushFlag( Flag* flag );
      Flag* PopFlag();
     
      int GetFlagCount() const { return flag_list.size(); }
   
      //      /** Add a pointer to a blinkenlight to the model. */
      //      void AddBlinkenlight( stg_blinkenlight_t* b )
      //      { g_ptr_array_add( this->blinkenlights, b ); }
   
      //      /** Clear all blinkenlights from the model. Does not destroy the
      //           blinkenlight objects. */
      //      void ClearBlinkenlights()
      //      {  g_ptr_array_set_size( this->blinkenlights, 0 ); }
   
      void Disable(){ disabled = true; };
 
      void Enable(){ disabled = false; };
   
      void LoadControllerModule( const char* lib );
     
      void NeedRedraw();
     
      void LoadBlock( Worldfile* wf, int entity );
 
      Block* AddBlockRect( stg_meters_t x, stg_meters_t y, 
                                  stg_meters_t dx, stg_meters_t dy, 
                                  stg_meters_t dz );
     
      void ClearBlocks();
   
      Model* Parent() const { return this->parent; }
 
      World* GetWorld() const { return this->world; }
   
      Model* Root(){ return(  parent ? parent->Root() : this ); }
   
      bool IsAntecedent( const Model* testmod ) const;
     
      bool IsDescendent( const Model* testmod ) const;
     
      bool IsRelated( const Model* testmod ) const;
 
      Pose GetGlobalPose() const;
     
      Velocity GetGlobalVelocity()  const;
     
      /* set the velocity of a model in the global coordinate system */
      void SetGlobalVelocity( const Velocity& gvel );
     
      void Subscribe();
     
      void Unsubscribe();
     
      void SetGlobalPose(  const Pose& gpose );
     
      void SetVelocity(  const Velocity& vel );
     
         void VelocityEnable();
 
         void VelocityDisable();
 
      void SetPose(  const Pose& pose );
     
      void AddToPose(  const Pose& pose );
     
      void AddToPose(  double dx, double dy, double dz, double da );
     
      void SetGeom(  const Geom& src );
   
      void SetFiducialReturn(  int fid );
   
      int GetFiducialReturn()  const { return vis.fiducial_return; }
   
      void SetFiducialKey(  int key );
     
      Color GetColor() const { return color; }
      
   uint32_t GetId()  const { return id; }
      
      stg_kg_t GetTotalMass();
      
      stg_kg_t GetMassOfChildren();
 
      int SetParent( Model* newparent);
     
      Geom GetGeom() const { return geom; }
     
      Pose GetPose() const { return pose; }
     
      Velocity GetVelocity() const { return velocity; }
     
      // guess what these do?
      void SetColor( Color col );
      void SetMass( stg_kg_t mass );
      void SetStall( stg_bool_t stall );
      void SetGravityReturn( int val );
      void SetGripperReturn( int val );
      void SetStickyReturn( int val );
      void SetLaserReturn( stg_laser_return_t val );
      void SetObstacleReturn( int val );
      void SetBlobReturn( int val );
      void SetRangerReturn( int val );
      void SetBoundary( int val );
      void SetGuiNose( int val );
      void SetGuiMove( int val );
      void SetGuiGrid( int val );
      void SetGuiOutline( int val );
      void SetWatts( stg_watts_t watts );
      void SetMapResolution( stg_meters_t res );
      void SetFriction( double friction );
     
      bool DataIsFresh() const { return this->data_fresh; }
     
      /* attach callback functions to data members. The function gets
          called when the member is changed using SetX() accessor method */
         
      void AddCallback( void* address, 
                              stg_model_callback_t cb, 
                              void* user );
         
      int RemoveCallback( void* member,
                                 stg_model_callback_t callback );
         
      int CallCallbacks(  void* address );
         
      /* wrappers for the generic callback add & remove functions that hide
          some implementation detail */
     
      void AddStartupCallback( stg_model_callback_t cb, void* user )
      { AddCallback( &hooks.startup, cb, user ); };
     
      void RemoveStartupCallback( stg_model_callback_t cb )
      { RemoveCallback( &hooks.startup, cb ); };
     
      void AddShutdownCallback( stg_model_callback_t cb, void* user )
      { AddCallback( &hooks.shutdown, cb, user ); };
     
      void RemoveShutdownCallback( stg_model_callback_t cb )
      { RemoveCallback( &hooks.shutdown, cb ); }
     
      void AddLoadCallback( stg_model_callback_t cb, void* user )
      { AddCallback( &hooks.load, cb, user ); }
     
      void RemoveLoadCallback( stg_model_callback_t cb )
      { RemoveCallback( &hooks.load, cb ); }
     
      void AddSaveCallback( stg_model_callback_t cb, void* user )
      { AddCallback( &hooks.save, cb, user ); }
     
      void RemoveSaveCallback( stg_model_callback_t cb )
      { RemoveCallback( &hooks.save, cb ); }
   
      void AddUpdateCallback( stg_model_callback_t cb, void* user )
      { AddCallback( &hooks.update, cb, user ); }
      
      void RemoveUpdateCallback( stg_model_callback_t cb )
      {  RemoveCallback( &hooks.update, cb ); }
      
      void AddFlagIncrCallback( stg_model_callback_t cb, void* user )
      {  AddCallback( &hooks.flag_incr, cb, user ); }
     
      void RemoveFlagIncrCallback( stg_model_callback_t cb )
      {  RemoveCallback( &hooks.flag_incr, cb ); }
 
      void AddFlagDecrCallback( stg_model_callback_t cb, void* user )
      {  AddCallback( &hooks.flag_decr, cb, user ); }
     
      void RemoveFlagDecrCallback( stg_model_callback_t cb )
   {  RemoveCallback( &hooks.flag_decr, cb ); }
      
      const void* GetProperty( const char* key ) const;
      bool GetPropertyFloat( const char* key, float* f, float defaultval ) const;     
      bool GetPropertyInt( const char* key, int* i, int defaultval ) const;   
      bool GetPropertyStr( const char* key, char** c, char* defaultval ) const;
 
      int SetProperty( const char* key, const void* data );
      void SetPropertyInt( const char* key, int i );
      void SetPropertyFloat( const char* key, float f );
      void SetPropertyStr( const char* key, const char* str );
      
      void UnsetProperty( const char* key );
         
      virtual void Print( char* prefix ) const;
      virtual const char* PrintWithPose() const;
     
      Pose GlobalToLocal( const Pose& pose ) const;
      
      Pose LocalToGlobal( const Pose& pose ) const
      {  
         return( ( GetGlobalPose() + geom.pose ) + pose );
      }
         
         void LocalToPixels( const std::vector<stg_point_t>& local,
                                                 std::vector<stg_point_int_t>& pixels) const;
         
      stg_point_t LocalToGlobal( const stg_point_t& pt) const;       
 
      Model* GetUnsubscribedModelOfType( const std::string& type ) const;
     
      Model* GetUnusedModelOfType( const std::string& type );
   
      bool Stalled() const { return this->stall; }
      
      unsigned int GetSubscriptionCount() const { return subs; }
 
      bool HasSubscribers() const { return( subs > 0 ); }     
 
      static std::map< std::string, creator_t> name_map;  
   };
 
 
   // BLOBFINDER MODEL --------------------------------------------------------
 
 
   class ModelBlobfinder : public Model
   {
   public:
      class Blob
   {
      public:
         Color color;
         uint32_t left, top, right, bottom;
         stg_meters_t range;
      };
 
      class Vis : public Visualizer 
      {
      private:
         //static Option showArea;
      public:
         Vis( World* world );
         virtual ~Vis( void ){}
         virtual void Visualize( Model* mod, Camera* cam );
      } vis;
 
   private:
      std::vector<Blob> blobs;
      std::vector<Color> colors;
 
      // predicate for ray tracing
      static bool BlockMatcher( Block* testblock, Model* finder );
      //static Option showBlobData;
 
   public:
      stg_radians_t fov;
      stg_radians_t pan;
      stg_meters_t range;
      unsigned int scan_height;
      unsigned int scan_width;
      
      // constructor
      ModelBlobfinder( World* world,
                             Model* parent,
                             const std::string& type );
      // destructor
      ~ModelBlobfinder();
     
      virtual void Startup();
      virtual void Shutdown();
      virtual void Update();
      virtual void Load();
         
      Blob* GetBlobs( unsigned int* count )
      { 
         if( count ) *count = blobs.size();
         return &blobs[0];
      }
 
      void AddColor( Color col );
 
      void RemoveColor( Color col );
 
      void RemoveAllColors();
   };
 
 
 
 
   // LASER MODEL --------------------------------------------------------
   
   class ModelLaser : public Model
   {
   public:
      class Sample
      {
      public:
         stg_meters_t range; 
         double reflectance; 
      };
         
      class Config
      {
      public:
         uint32_t sample_count; 
         uint32_t resolution; 
         Bounds range_bounds; 
         stg_radians_t fov; 
         stg_usec_t interval; 
      };
         
   private:   
      class Vis : public Visualizer 
      {
      private:
         static Option showArea;
         static Option showStrikes;
         static Option showFov;
         static Option showBeams;
 
      public:
         Vis( World* world );        virtual ~Vis( void ){}
      virtual void Visualize( Model* mod, Camera* cam );
      } vis;
         
      unsigned int sample_count;
      std::vector<Sample> samples;
 
      stg_meters_t range_max;
      stg_radians_t fov;
      uint32_t resolution;
     
      // set up data buffers after the config changes
      void SampleConfig();
 
   public:
      // constructor
      ModelLaser( World* world,
                      Model* parent,
                      const std::string& type ); 
   
      // destructor
      ~ModelLaser();
     
      virtual void Startup();
      virtual void Shutdown();
      virtual void Update();
      virtual void Load();
      virtual void Print( char* prefix );
   
      Sample* GetSamples( uint32_t* count );
      
      const std::vector<Sample>& GetSamples();
      
      Config GetConfig( );
      
      void SetConfig( Config& cfg );  
   };
   
 
   // Light indicator model
   class ModelLightIndicator : public Model
   {
   public:
      ModelLightIndicator( World* world, 
                                  Model* parent,
                                  const std::string& type );
      ~ModelLightIndicator();
   
      void SetState(bool isOn);
 
   protected:
      virtual void DrawBlocks();
 
   private:
      bool m_IsOn;
   };
 
   // \todo  GRIPPER MODEL --------------------------------------------------------
 
 
   class ModelGripper : public Model
   {
   public:
 
      enum paddle_state_t {
         PADDLE_OPEN = 0, // default state
         PADDLE_CLOSED, 
         PADDLE_OPENING,
         PADDLE_CLOSING,
      };
      
      enum lift_state_t {
         LIFT_DOWN = 0, // default state
         LIFT_UP, 
         LIFT_UPPING, // verbed these to match the paddle state
         LIFT_DOWNING, 
      };
      
      enum cmd_t {
         CMD_NOOP = 0, // default state
         CMD_OPEN, 
         CMD_CLOSE,
         CMD_UP, 
         CMD_DOWN    
      };
      
      
      struct config_t
      {
         Size paddle_size; 
         paddle_state_t paddles;
         lift_state_t lift;      
         double paddle_position; 
         double lift_position; 
         Model* gripped;
      bool paddles_stalled; // true iff some solid object stopped the paddles closing or opening
         double close_limit; 
         bool autosnatch; 
         double break_beam_inset[2]; 
       Model* beam[2]; 
       Model* contact[2]; 
      };
      
   private:
      virtual void Update();
      virtual void DataVisualize( Camera* cam );
      
      void FixBlocks();
      void PositionPaddles();
      void UpdateBreakBeams();
      void UpdateContacts();
 
      config_t cfg;
      cmd_t cmd;
      
      Block* paddle_left;
      Block* paddle_right;
 
      static Option showData;
 
   public:    
      static const Size size;
 
      // constructor
      ModelGripper( World* world,
                         Model* parent,
                         const std::string& type );
      // destructor
      virtual ~ModelGripper();
   
      virtual void Load();
      virtual void Save();
 
      void SetConfig( config_t & newcfg ){ this->cfg = newcfg; FixBlocks(); }
      
      config_t GetConfig(){ return cfg; };
      
      void SetCommand( cmd_t cmd ) { this->cmd = cmd; }
      void CommandClose() { SetCommand( CMD_CLOSE ); }
      void CommandOpen() { SetCommand( CMD_OPEN ); }
      void CommandUp() { SetCommand( CMD_UP ); }
      void CommandDown() { SetCommand( CMD_DOWN ); }
   };
 
 
   // \todo BUMPER MODEL --------------------------------------------------------
 
   //   typedef struct
   //   {
   //     Pose pose;
   //     stg_meters_t length;
   //   } stg_bumper_config_t;
 
   //   typedef struct
   //   {
   //     Model* hit;
   //     stg_point_t hit_point;
   //   } stg_bumper_sample_t;
 
 
   // FIDUCIAL MODEL --------------------------------------------------------
 
   class ModelFiducial : public Model
   {
   public:  
      class Fiducial
      {
      public:
         stg_meters_t range; 
         stg_radians_t bearing; 
         Pose geom; 
         Pose pose; 
         Model* mod; 
         int id; 
      };
 
   private:
      // if neighbor is visible, add him to the fiducial scan
      void AddModelIfVisible( Model* him );
 
      virtual void Update();
      virtual void DataVisualize( Camera* cam );
 
      static Option showData;
      static Option showFov;
      
   std::vector<Fiducial> fiducials;
         
   public:       
      ModelFiducial( World* world, 
                          Model* parent,
                          const std::string& type );
      virtual ~ModelFiducial();
     
      virtual void Load();
      void Shutdown( void );
 
      stg_meters_t max_range_anon;
      stg_meters_t max_range_id; 
      stg_meters_t min_range; 
      stg_radians_t fov; 
      stg_radians_t heading; 
      int key; 
         
         
      std::vector<Fiducial>& GetFiducials() { return fiducials; }
         
      Fiducial* GetFiducials( unsigned int* count )
      {
         if( count ) *count = fiducials.size();
         return &fiducials[0];
      }
   };
 
 
   // RANGER MODEL --------------------------------------------------------
 
   class ModelRanger : public Model
   {
   public:
      class Sensor
      {
      public:
         Pose pose;
         Size size;
         Bounds bounds_range;
         stg_radians_t fov;
         int ray_count;
         stg_meters_t range;
      };
 
   protected:
 
      virtual void Startup();
      virtual void Shutdown();
      virtual void Update();
      virtual void DataVisualize( Camera* cam );
     
   public:
      ModelRanger( World* world, Model* parent,
                       const std::string& type );
      virtual ~ModelRanger();
     
      virtual void Load();
      virtual void Print( char* prefix );
      
      std::vector<Sensor> sensors;
     
   private:
      static Option showRangerData;
      static Option showRangerTransducers;       
   };
     
   // BLINKENLIGHT MODEL ----------------------------------------------------
   class ModelBlinkenlight : public Model
   {
   private:
      double dutycycle;
      bool enabled;
      stg_msec_t period;
      bool on;
 
      static Option showBlinkenData;
   public:
      ModelBlinkenlight( World* world,
                               Model* parent,
                               const std::string& type );
 
      ~ModelBlinkenlight();
     
      virtual void Load();
      virtual void Update();
      virtual void DataVisualize( Camera* cam );
   };
 
     
   // CAMERA MODEL ----------------------------------------------------
 
   class ModelCamera : public Model
   {
   public:
      typedef struct 
   {
         // GL_V3F
         GLfloat x, y, z;
      } ColoredVertex;
   
   private:
      Canvas* _canvas;
 
      GLfloat* _frame_data;  //opengl read buffer
      GLubyte* _frame_color_data;  //opengl read buffer
 
      bool _valid_vertexbuf_cache;
      ColoredVertex* _vertexbuf_cache; //cached unit vectors with appropriate rotations (these must be scalled by z-buffer length)
     
      int _width;         //width of buffer
      int _height;        //height of buffer
      static const int _depth = 4;
     
      int _camera_quads_size;
      GLfloat* _camera_quads;
      GLubyte* _camera_colors;
     
      static Option showCameraData;
     
      PerspectiveCamera _camera;
      float _yaw_offset; //position camera is mounted at
      float _pitch_offset;
         
      bool GetFrame();
     
   public:
      ModelCamera( World* world,
                       Model* parent,
                       const std::string& type ); 
       
      ~ModelCamera();
   
      virtual void Load();
     
      virtual void Update();
     
      //virtual void Draw( uint32_t flags, Canvas* canvas );
     
      virtual void DataVisualize( Camera* cam );
     
      int getWidth( void ) const { return _width; }
     
      int getHeight( void ) const { return _height; }
     
      const PerspectiveCamera& getCamera( void ) const { return _camera; }
     
      const GLfloat* FrameDepth() const { return _frame_data; }
     
      const GLubyte* FrameColor() const { return _frame_color_data; }
     
      void setPitch( float pitch ) { _pitch_offset = pitch; _valid_vertexbuf_cache = false; }
     
      void setYaw( float yaw ) { _yaw_offset = yaw; _valid_vertexbuf_cache = false; }
   };
 
   // POSITION MODEL --------------------------------------------------------
 
   class ModelPosition : public Model
   {
      friend class Canvas;
 
   public:
      typedef enum
         { CONTROL_VELOCITY, 
           CONTROL_POSITION 
         } ControlMode;
      
      typedef enum
         { LOCALIZATION_GPS, 
           LOCALIZATION_ODOM 
         } LocalizationMode;
      
      typedef enum
         { DRIVE_DIFFERENTIAL, 
           DRIVE_OMNI, 
           DRIVE_CAR 
         } DriveMode;
      
   private:
      Pose goal;
   ControlMode control_mode;
      DriveMode drive_mode;
      LocalizationMode localization_mode; 
      Velocity integration_error; 
      
      
   public:
      // constructor
      ModelPosition( World* world,
                          Model* parent,
                          const std::string& type );
      // destructor
      ~ModelPosition();
 
      virtual void Startup();
      virtual void Shutdown();
      virtual void Update();
      virtual void Load();
         
      class Waypoint
      {
      public:
         Waypoint( stg_meters_t x, stg_meters_t y, stg_meters_t z, stg_radians_t a, Color color ) ;
         Waypoint( const Pose& pose, Color color ) ;
         Waypoint();
         void Draw() const;
         
         Pose pose;
         Color color;
      };
      
      std::vector<Waypoint> waypoints;
 
      class WaypointVis : public Visualizer
      {
      public:
         WaypointVis();
         virtual ~WaypointVis( void ){}
         virtual void Visualize( Model* mod, Camera* cam );
      } wpvis;
      
      class PoseVis : public Visualizer
      {
      public:
         PoseVis();
         virtual ~PoseVis( void ){}
         virtual void Visualize( Model* mod, Camera* cam );
      } posevis;
 
      void SetOdom( Pose odom );
         
      void SetSpeed( double x, double y, double a );
      void SetXSpeed( double x );
      void SetYSpeed( double y );
      void SetZSpeed( double z );
      void SetTurnSpeed( double a );
      void SetSpeed( Velocity vel );
      void Stop();
 
      void GoTo( double x, double y, double a );
      void GoTo( Pose pose );
 
      // localization state
      Pose est_pose; 
      Pose est_pose_error; 
      Pose est_origin; 
   };
 
 
   // ACTUATOR MODEL --------------------------------------------------------
 
   class ModelActuator : public Model
   {
   public:
      typedef enum
         { CONTROL_VELOCITY,
           CONTROL_POSITION
         } ControlMode;
   
      typedef enum
         { TYPE_LINEAR,
           TYPE_ROTATIONAL
         } ActuatorType;
   
   private:
      double goal; //< the current velocity or pose to reach, depending on the value of control_mode
      double pos;
      double max_speed;
      double min_position;
      double max_position;
      ControlMode control_mode;
      ActuatorType actuator_type;
      stg_point3_t axis;
   
      Pose InitialPose;
   public:  
      // constructor
    ModelActuator( World* world,
                          Model* parent,
                          const std::string& type );
      // destructor
      ~ModelActuator();
   
      virtual void Startup();
      virtual void Shutdown();
      virtual void Update();
      virtual void Load();
   
      void SetSpeed( double speed );
   
      double GetSpeed() const {return goal;}
   
      void GoTo( double pose );
   
      double GetPosition() const {return pos;};
      double GetMaxPosition() const {return max_position;};
      double GetMinPosition() const {return min_position;};
   
   };
 
 
 }; // end namespace stg
 
 #endif