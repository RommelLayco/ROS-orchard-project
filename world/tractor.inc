define tractorLaser ranger 
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

define tractor actor 
( 
	#body
  	model
	(
		size [2.4 0.8 0.6]
		pose [0 0 0.3 0.0]
		color "orange"
	)
	
	tractorLaser(pose [ 0.0 0.0 0.5 0.0 ])
	
	model
	(
		size [0.6 0.6 0.4]
		pose [ 0.5 0.0 0.9 0.0]
		color "orange"
	)

	model
	(
		size [ 0.4 0.4 0.1]
		pose [ 0.5 0.0 1.3 0.0]
		color "orange"
	)
	
	# exhaust
	model
	(
		size [ 0.1 0.1 0.4]
		pose [ 0.5 -0.1 1.4 0.0]
		color "black"
	)
	#seat
	model
	(
		size [ 0.5 0.5 0.3]
		pose [-0.6 0.0 0.9 0.0]
		color "yellow"
	)
	model
	(
		size [ 0.2 0.5 0.5]
		pose [-0.7 0.0 1.2 0.0]
		color "yellow"
	)
	# bottom right wheel
	model
	(
		size [ 0.7 0.15 0.7]
		pose [ -0.6 -0.5 0.2 0.0]
		color "black"
	)
	model
	(
		size [ 0.5 0.15 0.2]
		pose [ -0.6 -0.5 0.0 0.0]
		color "black"
	)
	model
	(
		size [ 0.5 0.15 0.2]
		pose [ -0.6 -0.5 0.9 0.0]
		color "black"
	)
	model
	(
		size [ 0.2 0.15 0.5]
		pose [ -0.2 -0.5 0.3 0.0]
		color "black"
	)
	model
	(
		size [ 0.2 0.15 0.5]
		pose [ -1.0 -0.5 0.3 0.0]
		color "black"
	)
	
	
	
	# bottom left wheel
	model
	(
		size [ 0.7 0.15 0.7]
		pose [ -0.6 0.5 0.2 0.0]
		color "black"
	)
	model
	(
		size [ 0.5 0.15 0.2]
		pose [ -0.6 0.5 0.0 0.0]
		color "black"
	)
	model
	(
		size [ 0.5 0.15 0.2]
		pose [ -0.6 0.5 0.9 0.0]
		color "black"
	)
	model
	(
		size [ 0.2 0.15 0.5]
		pose [ -0.2 0.5 0.3 0.0]
		color "black"
	)
	model
	(
		size [ 0.2 0.15 0.5]
		pose [ -1.0 0.5 0.3 0.0]
		color "black"
	)
	

	# upper right wheel
	model
	(
		size [ 0.7 0.15 0.7]
		pose [ 0.6 -0.5 0.2 0.0]
		color "black"
	)
	model
	(
		size [ 0.5 0.15 0.2]
		pose [ 0.6 -0.5 0.0 0.0]
		color "black"
	)
	model
	(
		size [ 0.5 0.15 0.2]
		pose [ 0.6 -0.5 0.9 0.0]
		color "black"
	)
	model
	(
		size [ 0.2 0.15 0.5]
		pose [ 0.2 -0.5 0.3 0.0]
		color "black"
	)
	model
	(
		size [ 0.2 0.15 0.5]
		pose [ 1.0 -0.5 0.3 0.0]
		color "black"
	)
	
	
	# upper left wheel
	model
	(
		size [ 0.7 0.15 0.7]
		pose [ 0.6 0.5 0.2 0.0]
		color "black"
	)
	model
	(
		size [ 0.5 0.15 0.2]
		pose [ 0.6 0.5 0.0 0.0]
		color "black"
	)
	model
	(
		size [ 0.5 0.15 0.2]
		pose [ 0.6 0.5 0.9 0.0]
		color "black"
	)
	model
	(
		size [ 0.2 0.15 0.5]
		pose [ 0.2 0.5 0.3 0.0]
		color "black"
	)
	model
	(
		size [ 0.2 0.15 0.5]
		pose [ 1.0 0.5 0.3 0.0]
		color "black"
	)
		
		
	
	
) 
