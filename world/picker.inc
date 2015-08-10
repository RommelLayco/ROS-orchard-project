

define picker actor 
( 
	#body
  	model
	(
		size [1 0.6 0.6]
		pose [0 0 0 0.0]
		color "pink"
	)
	
	myLaser(pose [ 0.0 0.0 0.5 0.0 ])
	
	#right arm
	model
	(
		size [0.8 0.1 0.15]
		pose [0.5 -0.3 0.5 0]
		color "white"
	)
	#left arm
	model
	(
		size [0.8 0.1 0.15]
		pose [0.5 0.3 0.5 0]
		color "white"
	)
	#left hand
	model
	(
		size [0.1 0.1 0.28]
		pose [0.95 -0.3 0.43 0]
		color "red"
	)
	
	model
	(
		size [0.32 0.1 0.1]
		pose [1 -0.3 0.43 0]
		color "red"
	)
	model
	(
		size [0.32 0.1 0.1]
		pose [1 -0.3 0.63 0]
		color "red"
	)

	#right hand
	model
	(
		size [0.1 0.1 0.28]
		pose [0.95 0.3 0.43 0]
		color "red"
	)
	
	model
	(
		size [0.32 0.1 0.1]
		pose [1 0.3 0.43 0]
		color "red"
	)
	model
	(
		size [0.32 0.1 0.1]
		pose [1 0.3 0.63 0]
		color "red"
	)
	
	#head
	model
	(
		size [0.3 0.3 0.3]
		pose [0 0 0.65 0]
		color "white"
	)
	model
	(
		size [0.3 0.20 0.05]
		pose [0 0 0.95 0]
		color "white"
	)
	
	#neck
	model
	(
		size [0.3 0.25 0.05]
		pose [0 0 0.60 0]
		color "white"
	)

	#right ear
	model
	(
		size [0.27 0.05 0.2]
		pose [0 -0.16 0.75 0]
		color "white"
	)
	model
	(
		size [0.2 0.05 0.15]
		pose [0 -0.2 0.75 0]
		color "white"
	)
	
	#left ear
	model
	(
		size [0.27 0.05 0.2]
		pose [0 0.16 0.75 0]
		color "white"
	)
	model
	(
		size [0.2 0.05 0.15]
		pose [0 0.2 0.75 0]
		color "white"
	)

	#right eye
	model
	(
		size [0.02 0.02 0.02]
		pose [0.18 -0.035 0.85 0]
		color "black"
	)
	
	#left eye
	model
	(
		size [0.02 0.02 0.02]
		pose [0.18 0.035 0.85 0]
		color "black"
	)

	#mouth
	model
	(
		size [0.02 0.15 0.01]
		pose [0.18 0 0.73 0]
		color "black"
	)
	
	model
	(
		size [0.01 0.01 0.07]
		pose [0.18 0.09 0.73 0]
		color "black"
	)
	model
	(
		size [0.01 0.01 0.07]
		pose [0.18 -0.09 0.73 0]
		color "black"
	)

	
	
	
	
	
	
	
	
	
	
) 
