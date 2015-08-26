define tractorWithWorkerLaser ranger 
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

define tractorWithWorker actor 
( 
	# Left Leg
	model
	(
		size [0.17 0.17 0.7]
		pose [-0.2 -0.13 0.8 0.0]
		color "blue"
	)

	# Right leg
	model
	(
		size [0.17 0.17 0.7]
		pose [-0.2 0.13 0.8 0.0]
		color "blue"
	)
	
	# Torso
	model
	(
		size [0.25 0.5 0.55]
		pose [-0.2 0.0 1.448 0.0] 
		color "white"
	)
	

	# Team 4 symbol
	# Horizontal
	model
	(
		size [0.01 0.01 0.2]
		pose [-0.05 0 1.8 0.0]
		color "black"
	)

	model
	(
		size [0.01 0.2 0.01]
		pose [-0.05 0.0 1.87 0.0]
		color "black"
	)
	
	model
	(
		size [0.01 0.02 0.01]
		pose [-0.05 0.0 2.0 0.0]
		color "black"
	)

	model
	(
		size [0.01 0.02 0.01]
		pose [-0.05 -0.01 1.99 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [-0.05 -0.02 1.98 0.0]
		color "bl0ck"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [-0.05 -0.03 1.97 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [-0.05 -0.04 1.96 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [-0.05 -0.05 1.95 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [-0.05 -0.06 1.94 0.0]
		color "black"
	)

	model
	(
		size [0.01 0.02 0.01]
		pose [-0.05 -0.07 1.93 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [-0.05 -0.08 1.92 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [-0.05 -0.09 1.91 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [-0.05 -0.10 1.90 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [-0.05 -0.11 1.89 0.0]
		color "black"
	)


	# Head
	model
	(
		size [0.25 0.18 0.03] 
		pose [-0.2 0.0 1.998 0.0] 
		color "ivory2"
	)
	
	model
	(
		size [0.25 0.25 0.3] 
		pose [-0.2 0.0 2.05 0.0] 
		color "PeachPuff1"
	)

	# Left eye
	model
	(
		size [0.01 0.05 0.05]
		pose [-0.075 0.0625 2.178 0.0]
		color "ivory2"
	)
	model
	(
		size [0.01 0.025 0.025]
		pose [-0.0735 0.0625 2.178 0.0]
		color "DodgerBlue"
	)

	# Right eye
	model
	(
		size [0.01 0.05 0.05]
		pose [-0.075 -0.0625 2.178 0.0]
		color "ivory2"
	)
	model
	(
		size [0.01 0.025 0.025]
		pose [-0.0735 -0.0625 2.178 0.0]
		color "DodgerBlue"
	)

	# Left sleeve
	model
	(
		size [0.12 0.12 0.6]
		pose [-0.2 0.31 1.398 0.0]
		color "white"
	)

	# Right sleeve
	model
	(
		size [0.12 0.12 0.6]
		pose [-0.2 -0.31 1.398 0.0]
		color "white"
	)

	# Left arm
	model
	(
		size [0.1 0.1 0.05]
		pose [-0.2 0.31 1.348 0.0]
		color "PeachPuff1"
	)

	# Right arm
	model
	(
		size [0.1 0.1 0.05]
		pose [-0.2 -0.31 1.48 0.0]
		color "PeachPuff1"
	)

	#body
  	model
	(
		size [2.4 0.8 0.6]
		pose [0 0 0.3 0.0]
		color "orange"
	)
	
	tractorWithWorkerLaser(pose [ 0.0 0.0 0.5 0.0 ])
	
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
