#generate 100 values for x, y, z coordinates
#Physics engine will create a new cube with these coordinates every 0.5 seconds (2 per second)

import random

#
def generatexyz(seed): #generates random values in a square in positive x, y, z of length 20 sides.
    random.seed(seed)
    xyzList = []
    xvalue = random.randint(0, 20)
    yvalue = random.randint(0, 20)
    zvalue = random.randint(2,22) #So none of the cubes start off inside of the ground. Still a range of 20m

    xyzList.append(xvalue) #length
    xyzList.append(yvalue) #depth
    xyzList.append(zvalue) #height

    return xyzList

#Text file to store values
outputFileHandler = open("cubePositions.txt", "a+")

#Creates the 100 values, writes to the file
for iterator in range(0, 100):
    positionValues = generatexyz(iterator)
    
    outputFileHandler.write(str(positionValues[0]) + "\t" + str(positionValues[1]) + "\t" + str(positionValues[2]) + "\n")


outputFileHandler.close()
