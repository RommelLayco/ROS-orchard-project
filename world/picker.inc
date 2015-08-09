define myLaser ranger 
( 
  sensor( range [ 0 20 ] fov 180.0 samples 180 ) 
  size [0.1 0.1 0.1] 
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

define picker actor 
( 
	#body
  	model
	(
		size [1.2 1.2 1]
		pose [0 0 0 0.0]
		color "blue"
	)
	#head
	model
	(
		size [0.5 0.5 0.3]
		pose [0 0 1 0]
		color "white"
	)
	#right arm
	model
	(
		size [1.2 0.1 0.15]
		pose [0.5 -0.62 0.5 0]
		color "white"
	)
	#left arm
	model
	(
		size [1.2 0.1 0.15]
		pose [0.5 0.62 0.5 0]
		color "white"
	)
	#left hand
	model
	(
		size [0.1 0.1 0.32]
		pose [1.16 -0.62 0.43 0]
		color "red"
	)
	
	model
	(
		size [0.2 0.1 0.1]
		pose [1.16 -0.62 0.43 0]
		color "red"
	)
	
	
	
	
) 
