define myLaser ranger 
( 
  sensor( range [ 0 3 ] fov 180 samples 180 ) 
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

define person1 actor
(

	# Left Leg
	model
	(
		size [0.17 0.17 0.700]
		pose [0.0 -0.13 0.0 0.0]
		color "blue"
	)

	# Right leg
	model
	(
		size [0.17 0.17 0.700]
		pose [0.0 0.13 0.0 0.0]
		color "blue"
	)
	
	# Torso
	model
	(
		size [0.25 0.5 0.55]
		pose [0.0 0.0 0.648 0.0] 
		color "white"
	)
	#laser position under Torso
	myLaser(pose [ 0.0 0.0 0.65 0.0 ])

	# Team 4 symbol
	# Horizontal
	model
	(
		size [0.01 0.01 0.2]
		pose [0.15 0 1.0 0.0]
		color "black"
	)

	model
	(
		size [0.01 0.2 0.01]
		pose [0.15 0.0 1.07 0.0]
		color "black"
	)
	
	model
	(
		size [0.01 0.02 0.01]
		pose [0.15 0.0 1.2 0.0]
		color "black"
	)

	model
	(
		size [0.01 0.02 0.01]
		pose [0.15 -0.01 1.19 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [0.15 -0.02 1.18 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [0.15 -0.03 1.17 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [0.15 -0.04 1.16 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [0.15 -0.05 1.15 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [0.15 -0.06 1.14 0.0]
		color "black"
	)

	model
	(
		size [0.01 0.02 0.01]
		pose [0.15 -0.07 1.13 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [0.15 -0.08 1.12 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [0.15 -0.09 1.11 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [0.15 -0.10 1.10 0.0]
		color "black"
	)
	model
	(
		size [0.01 0.02 0.01]
		pose [0.15 -0.11 1.09 0.0]
		color "black"
	)


	# Head
	model
	(
		size [0.25 0.18 0.03] 
		pose [0.0 0.0 1.198 0.0] 
		color "ivory2"
	)
	
	model
	(
		size [0.25 0.25 0.3] 
		pose [0.0 0.0 1.25 0.0] 
		color "PeachPuff1"
	)

	# Left eye
	model
	(
		size [0.01 0.05 0.05]
		pose [0.125 0.0625 1.378 0.0]
		color "ivory2"
	)
	model
	(
		size [0.01 0.025 0.025]
		pose [0.1265 0.0625 1.378 0.0]
		color "DodgerBlue"
	)

	# Right eye
	model
	(
		size [0.01 0.05 0.05]
		pose [0.125 -0.0625 1.378 0.0]
		color "ivory2"
	)
	model
	(
		size [0.01 0.025 0.025]
		pose [0.1265 -0.0625 1.378 0.0]
		color "DodgerBlue"
	)

	# Left sleeve
	model
	(
		size [0.12 0.12 0.6]
		pose [0.0 0.31 0.598 0.0]
		color "white"
	)

	# Right sleeve
	model
	(
		size [0.12 0.12 0.6]
		pose [0.0 -0.31 0.598 0.0]
		color "white"
	)

	# Left arm
	model
	(
		size [0.1 0.1 0.05]
		pose [0.0 0.31 0.548 0.0]
		color "PeachPuff1"
	)

	# Right arm
	model
	(
		size [0.1 0.1 0.05]
		pose [0.0 -0.31 0.548 0.0]
		color "PeachPuff1"
	)

	# Base
	size [0.0 0.0 0.0]
    drive "diff"
	color "ivory2"
	stack_children 0
	gui_nose 0
	obstacle_return 0
)
