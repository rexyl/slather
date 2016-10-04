g=g5
fps=50
all: compile

compile:
	javac slather/sim/*.java

gui:
	java slather.sim.Simulator --gui --fps ${fps} -g ${g}

run:
	java slather.sim.Simulator --fps ${fps} -g ${g}
