define pickerLaser ranger 
( 
  sensor( range [ 0 3 ] fov 60 samples 60 ) 
    size [0.100 0.100 0.100] 
      color "black" 
        block( 
            points 4 
                point[0] [0 0] 
                    point[1] [0.1 0] 
                        point[2] [0.1 0.1] 
                            point[3] [0 0.1] 
                                z [0 0.1] 
                                    ) 
        ) 

define masterNode actor
( 

	
	#body
	model
	(
		size [0.3 0.3 0.3]
		pose [0 0 -0.7 0.0]
		color "red"
	)
	bitmap "Circle.png"
	size [1 1 0.1]
	color "grey"

	#propeller
	model
	(
		size [0.1 0.1 0.8]
		pose [0 0 -0.7 0.0]
		color "grey"
	)


	#camera
	model
	(
		size [0.15 0.15 0.15]
		pose [0 0.15 -0.6 0.0]
		color "black"
	)

	model
	(
		size [0.08 0.08 0.08]
		pose [0 0.2 -0.55 0.0]
		color "white"
	)

	
#	model
#	(
#		size [0.1 0.6 0.03]
#		pose [0 0.3 1.3 0.0]
#		color "red"
#	)
#	model
#	(
#		size [0.1 0.6 0.03]
#		pose [0 -0.3 1.3 0.0]
#		color "red"
#	)
#	model
#	(
#		size [0.6 0.1 0.03]
#		pose [0.3 0 1.3 0.0]
#		color "red"
#	)
#	model
#	(
#		size [0.6 0.1 0.03]
#		pose [-0.3 0 1.3 0.0]
#		color "red"
#	)
#
#
	
	
	
) 
