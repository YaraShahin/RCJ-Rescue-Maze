""" # Using a Python dictionary to act as an adjacency list
graph = {
    'A' : ['B','C'],
    'B' : ['D', 'E'],
    'C' : ['F'],
    'D' : [],
    'E' : ['F'],
    'F' : []
}

visited = set() # Set to keep track of visited nodes.

def dfs(visited, graph, node):
    if node not in visited:
        print (node)
        
        visited.add(node)
        for neighbour in graph[node]:
            dfs(visited, graph, neighbour)

# Driver Code
dfs(visited, graph, 'A')

 """
#PseudoCode for DFS in Maze

#initialize adjacency list
""" mazeGraph = []
#initialize visited list
visitedTiles = [] """

#dfs function:
    #if node is not in visited
        #check right, left , forward and append to current node as neighbors with direction information
        #add node to visited list
        

        #loop over neighbors
            #Move to neighbor node and add movement information to object 
            #run dfs function
            
        #Backtrack using this movement information

""" graph = {
    "0": ["0N"],
    "0N": ["1N"],
    "1N": ["2N","2W"],
    "2N": ["4"]
} """

#########################################################################################################
import smbus
import time 

###Secondary Functions Start here


##Direction State Change Function and variables

#Initializing direction state to default (1)
directionState = 1

#Initializing direction graph (Maps integer state to side:direction map)
directionGraph = {
        "1":{
            "F":"N",
            "R":"E",
            "L":"W"
        },

        "2":{
            "F":"E",
            "R":"S",
            "L":"N"
        },

        "3":{
            "F":"S",
            "R":"W",
            "L":"E"
        },

        "4":{
            "F":"W",
            "R":"N",
            "L":"S"
        }
    }

#Direction Map is directionGraph[directionState]
directionMap = directionGraph[str(int(directionState))] 

#Function that updates direction state based on angle
def directionStateUpdate(angle):
    global directionState
    
    incr = angle/90
    
    directionState+=incr

    if directionState > 4:
        directionState =  directionState - 4

    if directionState < 1:
        directionState = directionState + 4

    currentMap = directionGraph[str(int(directionState))]
    #TESTING
    print("Updating current direction state...")
    print("New direction state is: " + str(directionState))
    print("New direction map is: ")
    print(currentMap)
    #print (directionState)
    #print (currentMap)

#Map that maps direction:coordinate transform index 0 is xTransform and index 1 is yTransform
coordTransformMap = {
        "N": [0,1],
        "S": [0,-1],
        "E": [1,0],
        "W": [-1,0],
    }

##Function that calculates required angle of rotation based on current direction and intended direction

def getAngleTransform(direction):
    #TESTING
    print("Calculating required angle for " + direction +" | Current state is: " + str(directionState))

    #Loops over direction graph and fetches index of intended direction
    for state in directionGraph:
        if directionGraph[state]["F"] == direction: #If the state currently looping through has the same direction as target direction
            offset = int(state) - directionState #Subtract intended state from current state
            #print(directionState)  
            #print(state)  
            break
    
    angleTransform = offset * 90 #Calculating Angle based on offset
    
    if angleTransform > 180: #Finding equivalent angle if angle > 180 (sometimes function returned 270 instead of -90)
        angleTransform-=360
    if angleTransform < -180: #Finding equivalent angle if angle < -180 (sometimes function returned -270 instead of 90)
        angleTransform+=360
    #print(angleTransform)

    #TESTING
    print("Calculated angle is : " + str(angleTransform))

    return angleTransform


##Function that sends specific byte to address 0x8 through i2c
def sendCommandi2c(byteCommand):
    bus = smbus.SMBus(1) #Defining i2c Bus 1
    address = 0x8 #Defining Arduino i2c Address

    bus.write_byte(address, byteCommand) 
    print(f"Sending {byteCommand} to arduino...")
    while True:
        if bus.read_byte(address) == 1:
            print("Action Completed")
            print("Byte is :",str(bus.read_byte(address)))
            break
        else:
            print("Action Still Running")
            print("Byte is :",str(bus.read_byte(address)))
        time.sleep(0.1)

#Mapping angles of rotation to bytes 
byteKey = {
    "0": 0,
    "90":2,
    "-90":3,
    "180":4,
}

##Move function
def move(currentNode,nextNode):
    #TESTING
    print("\n ----Move function----")
    print ("Starting movement from: " + currentNode +" to: " + nextNode)
    

    #Parsing string into integer array "-11" => [-1,1] for proper subtraction
    nextNode = parseCoord(nextNode)  
    currentNode = parseCoord(currentNode)  

    xTransform = nextNode[0] - currentNode[0] #Subtract x-coordinate of next from current

    yTransform = nextNode[1] - currentNode[1] #Subtract y-coordinate of next from current

    #TESTING
    print("xTransform is: "+ str(xTransform))
    print("yTransform is: "+ str(yTransform))

    if xTransform > 0:
        rotationAngle = getAngleTransform("E")
        
        #Sending Rotation command to Arduino
        byteCommand = byteKey[str(rotationAngle)]
        sendCommandi2c(byteCommand)
        
        #TESTING
        print("Rotation to angle " + str(rotationAngle) + " complete")
        directionStateUpdate(rotationAngle)
        
        #Sending movement command to Arduino
        sendCommandi2c(1)
    if xTransform < 0:
        rotationAngle = getAngleTransform("W")
        
        #Sending Rotation command to Arduino
        byteCommand = byteKey[str(rotationAngle)]
        sendCommandi2c(byteCommand)
        
        #TESTING
        print("Rotation to angle " + str(rotationAngle) + " complete")
        directionStateUpdate(rotationAngle)
        
        #Sending movement command to Arduino
        sendCommandi2c(1)
    
    if yTransform > 0:
        rotationAngle = getAngleTransform("N")
        
        #Sending Rotation command to Arduino
        byteCommand = byteKey[str(rotationAngle)]
        sendCommandi2c(byteCommand)
        
        #TESTING
        print("Rotation to angle " + str(rotationAngle) + " complete")
        directionStateUpdate(rotationAngle)
        
        #Sending movement command to Arduino
        sendCommandi2c(1)
    
    if yTransform < 0:
        rotationAngle = getAngleTransform("S")
        
        #Sending Rotation command to Arduino
        byteCommand = byteKey[str(rotationAngle)]
        sendCommandi2c(byteCommand)
        
        #TESTING
        print("Rotation to angle " + str(rotationAngle) + " complete")
        directionStateUpdate(rotationAngle)
        
        #Sending movement command to Arduino
        sendCommandi2c(1)


##Function that parses coordinates into two integer array
def parseCoord(coordinate): #"-11" => [-1,1]
    offset = 0
    if coordinate[0] == "-":
        xCoord = "-" + coordinate[1]
        offset+=1
    else:
        xCoord = coordinate[0]
    
    if coordinate[1+offset] == "-":
        offset +=1
        yCoord = "-" + coordinate[1+offset]
    else:
        yCoord = coordinate[1+offset]
    
    xCoord = int(xCoord)
    yCoord = int(yCoord)
    #print( [xCoord,yCoord])
    return [xCoord,yCoord]


##Victim Check placeholder function
def victimCheck(directions):
    print("\n ----Victim Check code----")
    if "R" in directions:
        print("Victim Check Right")
    if "L" in directions:
        print("Victim Check Left")
    if "F" in directions:
        print("Victim Check Front")
    print("") 

##Read Distance placeholder function 
def readDistance(direction):
    if direction == "R":
        x = input("Enter distance for right sensor: ")
    if direction == "L":
        x = input("Enter distance for left sensor: ")
    if direction == "F":
        x = input("Enter distance for front sensor: ")
    
    return int(x)

###Main Algorithm Starts here

#initialize adjacency list
mazeGraph = {}


#initialize visited list
visitedTiles = []

#initialize node counter variable
xCoord = 0
yCoord = 0
#nodeName = str(xCoord)+str(yCoord) #e.g: 00 , 01 , 11...
origin = "00"

def blindDFS(visited,graph,node):
    directionMap = directionGraph[str(int(directionState))]
    #TESTING
    print("-----------------------------------------------------------")
    print("-----------------------------------------------------------")
    print("Current Node is: " + node)
    if node not in visited:
        #TESTING
        print("Node: "+ node + "  | not visited")
        print("Graph is: ")
        print(graph)
        print("Current direction map is: ")
        print(directionMap)

        #Initialize victim direction state
        victimDirection = ""
        frontVictimCheck = 0

        #Get Current coordinates of point from node name
        nodeArr = parseCoord(node) 
        xCoord = nodeArr[0]
        yCoord = nodeArr[1]

        #Add current node to the list of visited nodes
        visited.append(node) 

        #Add node element to graph object as an array
        graph[node] = []
        
        #Get Distance readings from ultrasonic sensors
        rightDistance = readDistance("R")                                                                              
        leftDistance = readDistance("L")                                                                              
        frontDistance = readDistance("F")
        
        #Determine if there are any vacant neighboring tiles and add to node element array
        if rightDistance > 10:
            rightSideDirection = directionMap["R"]
            transformArr = coordTransformMap[rightSideDirection]
            graph[node].append( str(xCoord + transformArr[0]) + str(yCoord + transformArr[1]))
            #TESTING
            print ("Empty tile detected right side, node added to graph")
            print("Empty tile direction is: "+ rightSideDirection)
            print("Transformation array is: ")
            print(transformArr)

            print("New Maze graph is: ")
            print (graph)  
        else:
            victimDirection+= "R" 
        
        if leftDistance > 10:
            leftSideDirection = directionMap["L"]
            transformArr = coordTransformMap[leftSideDirection]
            graph[node].append( str(xCoord + transformArr[0]) + str(yCoord + transformArr[1]))
            #TESTING
            print ("Empty tile detected left side, node added to graph")
            print("Empty tile direction is: "+ leftSideDirection)
            print("Transformation array is: ")
            print(transformArr)

            print("New Maze graph is: ")
            print (graph)
        else:
            victimDirection+= "L" 
        
        if frontDistance > 30:
            frontSideDirection = directionMap["F"]
            transformArr = coordTransformMap[frontSideDirection]
            graph[node].append( str(xCoord + transformArr[0]) + str(yCoord + transformArr[1]))
            #TESTING
            print ("Empty tile detected front side, node added to graph")
            print("Empty tile direction is: "+ frontSideDirection)
            print("Transformation array is: ")
            print(transformArr)

            print("New Maze graph is: ")
            print (graph) 
        else:
            frontVictimCheck+= 1 
        
        #Perform Victim Check on sides
        victimCheck(victimDirection)

        #Check front wall for victims by rotating -90 and checking for right side
        if frontVictimCheck == 1:
            sendCommandi2c(3) #Rotating -90
            directionStateUpdate(-90)
            victimCheck("R")
    
        #Loop over neighboring tiles
        for neighbor in graph[node]: #03, -12
            #Check if node is visited before moving to node
            if neighbor in visited:
                continue
            else:
                #Move to node and record movement information (direction/encoder data or is direction enough?)
                #robot.move(node,neighbor) 17,27 20,000+ , 20,000
                move(node,neighbor) #02 , -12 

                #Call dfs function again
                blindDFS(visited,graph,neighbor)

                #Move back to previous tile after visiting neighbors
                #reverse()/ move(neighbor,node,1(optional argument for return))
                #TESTING
                print("\n ----RETURNING----")
                print("Moving back from node " + neighbor + " to node " +node)
                move(neighbor,node) #Required, since you might turn

blindDFS(visitedTiles,mazeGraph,origin) #Call the path finding algorithm
