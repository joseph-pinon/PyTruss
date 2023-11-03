
import numpy as np
import sympy
import tkinter as tk

class Window():
    def __init__(self, nodeSize = 5, lineThickness = 3, offset = (10,10), fontSize = 20, width = 500, height = 500):
        self.window = tk.Tk()
        self.window.title("Truss Simulator")
        self.window.geometry(f"{width}x{height}")
        self.window.update()

        self.canvas = tk.Canvas(self.window, width = width, height = height)
        self.canvas.pack()

        self.nodeSize = nodeSize
        self.lineThickness = lineThickness
        self.fontSize = fontSize
        self.offsetX = offset[0]
        self.offsetY = offset[1]

        self.halfWidth = width/2
        self.halfHeight = height/2

        self.canvas.bind("<Configure>", self.on_resize)

    def on_resize(self,event):
        #self.height = self.winfo_reqheight()
        #self.width = self.winfo_reqwidth()

        # determine the ratio of old width/height to new width/height

        self.width = event.width
        self.height = event.height
        # resize the canvas 
        self.canvas.config(width=self.width, height=self.height)

    def create_circle(self, x, y, r): #center coordinates, radius
        x0 = x - r
        y0 = y - r
        x1 = x + r
        y1 = y + r
        self.canvas.create_oval(x0, y0, x1, y1, fill = "black")

    def create_line(self, x0, x1, y0, y1, t, color):
        self.canvas.create_line(x0, y0, x1, y1, width = t, fill = color)
    
    def create_label(self, x, y, force):
        self.canvas.create_text(x, y, text=force, fill="black", font=("Arial", self.fontSize))

    def Remap(self, value, low1, high1, low2, high2):
        return low2 + (value - low1) * (high2 - low2) / (high1 - low1)

    def DrawNodes(self, graph):
        Xmin, Xmax, Ymin, Ymax = graph.GetTrussDimensions()

        ## Go through nodes
        for node in graph.nodes:
            x = self.Remap(node.x, Xmin, Xmax, self.window.winfo_width() - self.offsetX, self.offsetX)
            y = self.Remap(node.y, Ymin, Ymax, self.window.winfo_height() - self.offsetY,  self.offsetY)

            self.create_circle(x, y, self.nodeSize)
    
    def DrawConnections(self, graph, solution = None):
        Xmin, Xmax, Ymin, Ymax = graph.GetTrussDimensions()

        ## Go Through connections
        for connection in graph.connections:
            nodeA = connection.nodeA
            nodeB = connection.nodeB

            x0 = self.Remap(nodeA.x, Xmin, Xmax, self.offsetX, self.window.winfo_width() - self.offsetX)
            x1 = self.Remap(nodeB.x, Xmin, Xmax, self.offsetX, self.window.winfo_width() - self.offsetX)
            y0 = self.Remap(nodeA.y, Ymin, Ymax, self.window.winfo_height() - self.offsetY, self.offsetY)
            y1 = self.Remap(nodeB.y, Ymin, Ymax, self.window.winfo_height() - self.offsetY, self.offsetY)

            color = "black"

            if (type(solution) != None):
                force = solution[connection.id, :]
                
                if (force < 0):
                    color = "red"    
                else:
                    color = "blue"
                
                self.create_line(x0, x1, y0, y1, self.lineThickness, color)
                self.create_label((x0 + x1)/2, (y0 + y1)/2, np.abs(int(force)))

            else:
                self.create_line(x0, x1, y0, y1, self.lineThickness, color)

    def Draw(self, graph, solution = None):

        self.graph = graph
        self.solution = solution
        self.ReDraw()
        
    def ReDraw(self):
        self.canvas.delete("all")
        self.DrawConnections(self.graph, self.solution)
        self.DrawNodes(self.graph)
        self.window.after(50, self.ReDraw)  
          

class Node():

    def __init__(self, x, y, reactions = (0, 0), force = (0, 0)):
        self.id = 0
        self.x = x
        self.y = y
        self.connections = []
        self.reactions = reactions
        self.reactionCount = reactions[0] + reactions[1]
        self.force = force

    def SetID(self, id):
        self.id = id

    def AddConnection(self, connection):
        self.connections.append(connection)

    def CalculateAngle(self):

        if (self.force[0] == 0 and self.force[1] > 0):
            return np.pi/2       
        elif (self.force[0] == 0 and self.force[1] < 0):
            return -np.pi/2   
        else:
            return np.arctan2(self.force[1], self.force[0])   

class Connection():
    def __init__(self, nodeA, nodeB):
        self.id = 0

        self.NotifyNodes(nodeA, nodeB)

        self.nodeA = nodeA
        self.nodeB = nodeB

    def SetID(self, id):
        self.id = id
        
    def CalculateAngle(self, node1, node2):

        dx = node1.x - node2.x
        dy = node1.y - node2.y

        if (dx == 0 and dy > 0):
            return np.pi/2
        elif (dx == 0 and dy < 0):
            return -np.pi/2
        else:
            return np.arctan2(dy, dx)

    def NotifyNodes(self, nodeA, nodeB):
        nodeA.AddConnection(self)
        nodeB.AddConnection(self)

    def GetOtherNode(self, nodeStart):
        if (nodeStart == self.nodeA):
            return self.nodeB
        else:
            return self.nodeA

    def GetXFactor(self, nodeStart):
        other = self.GetOtherNode(nodeStart)
        angle = self.CalculateAngle(nodeStart, other)
        return np.cos(angle)
    
    def GetYFactor(self, nodeStart):
        other = self.GetOtherNode(nodeStart)
        angle = self.CalculateAngle(nodeStart, other)
        return np.sin(angle)  

class Graph():
    def __init__(self):
        self.nodes = []
        self.connections = []
        self.nodeCount = 0
        self.connectionCount = 0
        self.unknownsCount = 0
    
    def AddNode(self, node):
        node.SetID(self.nodeCount)
        self.nodes.append(node)
        self.nodeCount += 1

        self.unknownsCount += node.reactionCount
         
    def AddConnection(self, connection):
        connection.SetID(self.connectionCount)
        self.connections.append(connection)
        self.connectionCount += 1
        self.unknownsCount += 1

    def GetTrussDimensions(self):
        
        xArray = np.zeros(self.nodeCount)
        yArray = np.zeros(self.nodeCount)

        for i in range(self.nodeCount):
            node = self.nodes[i]
            xArray[i] = node.x
            yArray[i] = node.y

        Xmin = np.min(xArray)
        Xmax = np.max(xArray)

        Ymin = np.min(yArray)
        Ymax = np.max(yArray)

        return Xmin, Xmax, Ymin, Ymax
        
    def Solve(self):
        self.reactionCount = self.GetReactionCount()
        self.solutionCount = self.connectionCount + self.reactionCount

        self.A, self.b = self.GetMatrix()

        rank = np.linalg.matrix_rank(self.A) 
        
        if (rank > self.unknownsCount):
            print("SYSTEM OVERDETERMINED! - Too many Force/Moment Balances")
            return

        elif (rank < self.unknownsCount):
            print("SYSTEM UNDER-DETERMINED! - Not enough Force/Moment Balances")
            return
        
        return np.linalg.solve(self.A, self.b)
  
    def GetReactionCount(self):
        
        reactionCount = 0
        for node in self.nodes:
            reactionCount += node.reactionCount

        return reactionCount
         
    def GetMatrix(self):

        A = np.zeros((self.nodeCount * 2 + 3, self.solutionCount))
        b = np.zeros((self.nodeCount * 2 + 3, 1))
        
        connectionEquationIndex = 0
        reactionEquationIndex = self.connectionCount

        xReactionEquation = np.zeros(self.solutionCount)
        yReactionEquation = np.zeros(self.solutionCount)
        
        torqueReactionEquation = np.zeros(self.solutionCount)
        referencePosX = self.nodes[0].x
        referencePosY = self.nodes[0].y

        totalForceX = 0
        totalForceY = 0
        totalTorque = 0

        #Look through all the connections in the node
        for node in self.nodes:

            #Build 2 vectors representing node for each direction
            xEquation = np.zeros(self.solutionCount)
            yEquation = np.zeros(self.solutionCount)

            for connection in node.connections:         
                #X
                xEquation[connection.id] = connection.GetXFactor(node)
                
                #Y
                yEquation[connection.id] = connection.GetYFactor(node)
     
            #External forces
            if (node.reactions[0] == 1):
                xReactionEquation[reactionEquationIndex] = 1
                xEquation[reactionEquationIndex] = 1

                torqueReactionEquation[reactionEquationIndex] = node.y - referencePosY

                reactionEquationIndex += 1

            if (node.reactions[1] == 1):
                yReactionEquation[reactionEquationIndex] = 1
                yEquation[reactionEquationIndex] = 1

                torqueReactionEquation[reactionEquationIndex] = node.x - referencePosX


                reactionEquationIndex += 1

            #Fill vector in a the proper location in the matrix
            A[connectionEquationIndex] = xEquation
            A[connectionEquationIndex + 1] = yEquation

            b[connectionEquationIndex] = node.force[0]
            b[connectionEquationIndex + 1] = node.force[1]

            #Add to total force
            totalForceX += node.force[0]
            totalForceY += node.force[1]

            #Add to total torque due to forces
            totalTorque += node.force[0] * (node.y - referencePosY)
            totalTorque += node.force[1] * (node.x - referencePosX)

            connectionEquationIndex += 2

        A[connectionEquationIndex] = xReactionEquation
        b[connectionEquationIndex] = totalForceX

        A[connectionEquationIndex + 1] = yReactionEquation
        b[connectionEquationIndex + 1] = totalForceY

        A[connectionEquationIndex + 2] = torqueReactionEquation
        b[connectionEquationIndex + 2] = totalTorque

        #Need to reduce A matrix

        _, indexes = sympy.Matrix(A).T.rref()
        A = A[indexes, :]
        b = b[indexes, :]

        return A, b
    
class Solver():
    def __init__(self, nodeSize = 5, lineThickness = 3, offset = (50,50), fontSize = 20, width = 500, height = 500):
        self.graph = Graph()
        self.window = Window(nodeSize, lineThickness, offset, fontSize, width, height)
        self.solution = None

    def AddNodes(self, nodes):
        for node in nodes:
            self.graph.AddNode(node)
    
    def AddConnections(self, connections):
        for connection in connections:
            self.graph.AddConnection(connection)

    def AddConnection(self, connection):
        self.graph.AddConnection(connection)

    def AddNode(self, node):
        self.graph.AddNode(node)

    def Solve(self):
        self.solution = self.graph.Solve()
        return self.solution
    
    def Draw(self):
        self.window.Draw(self.graph, self.solution)
        self.window.window.mainloop()





    
