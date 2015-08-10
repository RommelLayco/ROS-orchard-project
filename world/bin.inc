define bin model 
(         
        model
        (
            size [0.42 0.02 0.4 ]
            pose [0.0 0.2 0.0 0.0]
            color "burlywood"
        )

        model
        (
            size [0.42 0.02 0.4 ]
            pose [0.0 -0.2 0.0 0.0]
            color "burlywood"
        )

        model
        (
            size [0.02 0.36 0.4 ]
            pose [0.2 0.0 0.0 0.0]
            color "burlywood"
        )

        model
        (
            size [0.02 0.36 0.4 ]
            pose [-0.2 0.0 0.0 0.0]
            color "burlywood"
        )

        model
        (
            size [0.4 0.4 0.02 ]
            pose [0.0 0.0 0.0 0.0]
            color "burlywood"
        )

        # Base
	size [0 0 0]
	color "burlywood"
	stack_children 0
	gui_nose 0
	obstacle_return 0
)
