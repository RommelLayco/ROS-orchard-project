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

define carrier actor
( 
	#body
  	model
	(
		size [1 0.6 0.4]
		pose [0 0 0 0.0]
		color "orange"
	)
	
	pickerLaser(pose [ 0.0 0.0 0.5 0.0 ])
	
	#right arm
	model
	(
		size [0.15 0.1 0.7]
		pose [0 -0.3 0.3 0]
		color "white"
	)
	#left arm
	model
	(
		size [0.15 0.1 0.7]
		pose [0 0.3 0.3 0]
		color "white"
        )
	
	#head
	model
	(
		size [0.3 0.3 0.3]
		pose [0 0 0.45 0]
		color "white"
	)
	model
	(
		size [0.3 0.20 0.05]
		pose [0 0 0.75 0]
		color "white"
	)
	
	#neck
	model
	(
		size [0.3 0.25 0.05]
		pose [0 0 0.40 0]
		color "white"
	)

	#right ear
	model
	(
		size [0.27 0.05 0.2]
		pose [0 -0.16 0.55 0]
		color "white"
	)
	model
	(
		size [0.2 0.05 0.15]
		pose [0 -0.2 0.55 0]
		color "white"
	)
	
	#left ear
	model
	(
		size [0.27 0.05 0.2]
		pose [0 0.16 0.55 0]
		color "white"
	)
	model
	(
		size [0.2 0.05 0.15]
		pose [0 0.2 0.55 0]
		color "white"
	)

	#right eye
	model
	(
		size [0.02 0.02 0.02]
		pose [0.18 -0.035 0.65 0]
		color "black"
	)
	
	#left eye
	model
	(
		size [0.02 0.02 0.02]
		pose [0.18 0.035 0.65 0]
		color "black"
	)

	#mouth
	model
	(
		size [0.02 0.15 0.01]
		pose [0.18 0 0.53 0]
		color "black"
	)
	
	model
	(
		size [0.01 0.01 0.07]
		pose [0.18 0.09 0.53 0]
		color "black"
	)
	model
	(
		size [0.01 0.01 0.07]
		pose [0.18 -0.09 0.53 0]
		color "black"
	)

	
	
	
	
	
	
	
	
	
	
) 
