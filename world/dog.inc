define dog actor
(

	# right front Leg
	model
	(
		size [0.17 0.1 0.4]
		pose [0.0 -0.13 0.1 0.0]
		color "yellow"
	)
	model
	(
		size [0.17 0.11 0.1]
		pose [0.0 -0.13 0 0.0]
		color "black"
	)
	
	#right back Leg
	model
	(
		size [0.17 0.1 0.4]
		pose [0 0.13 0.1 0.0]
		color "yellow"
	)
	model
	(
		size [0.17 0.11 0.1]
		pose [0 0.13 0 0.0]
		color "black"
	)

	#left front Leg
	model
	(
		size [0.17 0.1 0.4]
		pose [-0.4 -0.13 0.1 0.0]
		color "yellow"
	)
	model
	(
		size [0.17 0.11 0.1]
		pose [-0.4 -0.13 0 0.0]
		color "black"
	)
	
	#left back Leg
	model
	(
		size [0.17 0.1 0.4]
		pose [-0.4 0.13 0.1 0.0]
		color "yellow"
	)
	model
	(
		size [0.17 0.11 0.1]
		pose [-0.4 0.13 0 0.0]
		color "black"
	)

	#body
	model
	(
		size [0.8 0.17 0.2]
		pose [-0.30 0 0.3 0.0]
		color "yellow"
	)

	#tail
	model
	(
		size [0.15 0.13 0.15]
		pose [-0.65 0 0.5 0]
		color "yellow"
	)
	
	

	#collar
	model
	(
		size [0.17 0.11 0.15]
		pose [0 0 0.48 0.0]
		color "red"
	)

	#neck
	model
	(
		size [0.17 0.11 0.3]
		pose [0 0 0.6 0.0]
		color "yellow"
	)
	
	#ears
	model
	(
		size [0.17 0.3 0.15]
		pose [0 0 0.85 0.0]
		color "yellow"
	)
	#face
	model
	(
		size [0.12 0.15 0.3]
		pose [0.13 0 0.7 0.0]
		color "yellow"
	)
	#nose
	model
	(
		size [0.17 0.11 0.1]
		pose [0.25 0 0.7 0.0]
		color "black"
	)
	
	myLaser(pose [ 0.25 0.0 0.7 0.0 ])
	
	#eyes
	# Left eye
	model
	(
		size [0.01 0.05 0.05]
		pose [0.20 0.05 0.9 0.0]
		color "ivory2"
	)
	model
	(
		size [0.01 0.025 0.025]
		pose [0.20 0.05 0.9 0.0]
		color "DodgerBlue"
	)

	# Right eye
	model
	(
		size [0.01 0.05 0.05]
		pose [0.20 -0.05 0.9 0.0]
		color "ivory2"
	)
	model
	(
		size [0.01 0.025 0.025]
		pose [0.20 -0.05 0.9 0.0]
		color "DodgerBlue"
	)

)
