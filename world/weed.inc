define weed model
(

	#base
	bitmap "Circle.png"
	size [0.08 0.08 0.25 0]
	pose [0 0 0 0]
	color "green"

	#leaves base
	model
	(
		size [0.01 0.01 0.75]
		pose [0 0 -0.1 0]
		color "green"
	)
	model
	(
		size [0.01 0.01 0.75]
		pose [0 0.08 -0.1 0]
		color "green"
	)
	model
	(
		size [0.01 0.01 0.75]
		pose [0.08 0 -0.1 0]
		color "green"
	)
	model
	(
		size [0.01 0.01 0.75]
		pose [0 -0.08 -0.1 0]
		color "green"
	)
	model
	(
		size [0.01 0.01 0.75]
		pose [-0.08 0 -0.1 0]
		color "green"
	)

	#extend leaves
	model
	(
		size [0.01 0.3 0.01]
		pose [0 0.08 0.5 0]
		color "green"	
	)
	model
	(
		size [0.01 0.3 0.01]
		pose [0.08 0 0.3 120]
		color "green"	
	)


	

)
