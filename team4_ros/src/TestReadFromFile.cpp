#include <iostream>
#include <stdlib.h>
	#include <fstream>
	#include <sstream>
	using namespace std;
	 
	const int ROWS = 4;
	const int COLS = 2;
	const int BUFFSIZE = 80;
	 
	int main() {
	  float positions[ROWS][COLS];
	  char buff[BUFFSIZE]; // a buffer to temporarily park the data
	  ifstream infile("../../locations/tractorLocations");
	  stringstream ss;
	  for( int row = 0; row < ROWS; ++row ) {
	    // read a full line of input into the buffer (newline is
	    //  automatically discarded)
	    infile.getline( buff,  BUFFSIZE );
	    // copy the entire line into the stringstream
	    ss << buff;
	    for( int col = 0; col < COLS; ++col ) {
	     
	      ss.getline( buff, 6, ' ' );
	    
	      positions[row][col] = atof( buff );
	    }
	    
	    ss << "";

	    ss.clear();
  }
	  // Now print the array to see the result
		cout << positions[0][0];
		cout << positions[0][1];
	  /*for( int row = 0; row < ROWS; ++row ) {
	    for( int col = 0; col < COLS; ++col ) {
      		cout << array[row][col] << " ";
	    }
	    cout << endl;
	  }*/
	  infile.close();
	}
