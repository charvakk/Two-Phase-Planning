import subprocess
import sys
import os
from xml.etree import ElementTree

def main():
	# fileName = sys.argv[1]
	global expNumber
	expNumber = 0
	expFile = 'ck_diffusion.argos'
	global tree
	try:
	   tree = ElementTree.parse('src/experiments/' + expFile)   
	except Exception:
		pass

	root = tree.getroot()
	global exp
	exp = root[0][1]
	global lf
	lf = root[2]

	tuplePosition = ['center', 'corner']
	numOfRobots = [10, 50, 100]
	density = [2, 0.5, 1]
	k_Rps = [2, 4, 6]

	for pos in tuplePosition:
		for num in numOfRobots:
			for dens in density:
				for k in k_Rps:
					RunExperiment(expFile, 10, pos, num, dens, k)


def RunExperiment(expFile, numOfRepetitions, tuplePosition, numOfRobots, density, k_Rp):
	"""Runs the experiment with the specified parameters for the specified times."""
	global expNumber
	expNumber += 1
	for k in range(0, numOfRepetitions):
		i = k+1
		print str(expNumber) + '.' + str(i)
		exp.set('random_seed', str(i+27))
		expName = str(tuplePosition) + '_' + str(numOfRobots) + 'R_' + str(density) + 'D'
		# lf.set('outfile', expName + str(i) + '.dat')
		lf.set('outfile', 'justTheLifetimeWithRp.dat')
		lf.set('tuple_position', tuplePosition)
		lf.set('robots', str(numOfRobots))
		lf.set('density', str(density))
		lf.set('k_Rp', str(k_Rp))

		tree.write('src/experiments/' + expFile)
		command = 'argos3 -c src/experiments/' + expFile	
		with cd('/home/charvak/argos/NESTresearch'):
			subprocess.call(command, shell=True)

class cd:
    """Context manager for changing the current working directory"""
    def __init__(self, newPath):
        self.newPath = os.path.expanduser(newPath)

    def __enter__(self):
        self.savedPath = os.getcwd()
        os.chdir(self.newPath)

    def __exit__(self, etype, value, traceback):
        os.chdir(self.savedPath)


if __name__ == '__main__':
	main()