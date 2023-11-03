
import numpy as np
import sympy
import tkinter as tk
import math

class Window():
    def __init__(self, nodeSize, lineThickness, offset, fontSize, width, height, scale):
        self.root = tk.Tk()
        self.root.title("Truss Simulator")
        self.root.geometry(f"{width}x{height}")
        self.root.update()

        self.canvas = tk.Canvas(self.root, width = width, height = height)
        self.canvas.pack()

        self.nodeSize = nodeSize
        self.lineThickness = lineThickness
        self.fontSize = fontSize
        self.offsetX = offset[0]
        self.offsetY = offset[1]
        self.scale = scale

        self.halfWidth = width/2
        self.halfHeight = height/2

        self.canvas.bind("<Configure>", self.on_resize)

    def on_resize(self,event):
        self.width = event.width
        self.height = event.height

        # resize the canvas 
        self.canvas.config(width=self.width, height=self.height)
        self.ReDraw()

    def create_circle(self, x, y, r, fill = "black", outline = "black"): #center coordinates, radius
        x0 = x - r
        y0 = y - r
        x1 = x + r
        y1 = y + r
        self.canvas.create_oval(x0, y0, x1, y1, fill = fill, outline = outline)

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
            x = self.Remap(node.x, Xmin, Xmax, self.root.winfo_width() - self.offsetX, self.offsetX)
            y = self.Remap(node.y, Ymin, Ymax, self.root.winfo_height() - self.offsetY,  self.offsetY)

            self.create_circle(x, y, self.nodeSize)

            if (node.forceApplied):
                self.DrawExternalForces(node.force, x, y, "black", self.lineThickness, self.scale)

            if (node.reactionCount > 0):
                self.DrawSupport(x, y, node.reactions, self.scale, "black")

                if (type(self.solution) != type(None)):
                    self.DrawReactionForces(x, y, self.scale, node)


    def DrawForceVector(self, forceX, forceY, x, y, color, thickness, scale):

        angle = math.atan2(forceY, forceX)

        endX = x + math.cos(angle) * scale 
        endY = y - math.sin(angle) * scale

        self.create_line(x, endX, y, endY, thickness, color)

        ##Draw Arrow
        factor = 1/7

        arrowHeadX = endX
        arrowHeadY = endY - 10 * math.sin(angle)

        arrowEndX1 = endX - scale * factor * math.cos(angle)
        arrowEndX2 = endX + scale * factor * math.cos(angle)
        arrowEndY = endY - scale * factor * math.sin(angle)

        #self.canvas.create_polygon(arrowHeadX, arrowHeadY, arrowEndX1, arrowEndY, arrowEndX2, arrowEndY, fill = color)

        self.create_circle(arrowHeadX, arrowEndY, 1)
        
        offset = 10
        
        textX = arrowHeadX  + math.cos(angle) * offset
        textY = arrowHeadY  - math.sin(angle) * offset
        self.create_label(textX, textY, f'({forceX}, {forceY})')

    def DrawSupport(self, x, y, reactions, scale, color):
        factor = 0.3
        if (reactions[0] == True):
            if (reactions[1] == True):
                self.DrawPinnedSupport(x, y, scale, factor, color)
            else:
                self.DrawRollerY(x, y, scale, factor, color)
        else:
            self.DrawRollerX( x, y, scale, factor, color)
                
    
    def DrawPinnedSupport(self, x, y, scale, factor, color):
        
        x1 = x + scale * factor
        yNew = y + scale * factor

        x2 = x - scale * factor

        self.canvas.create_polygon(x, y, x1, yNew, x2, yNew, fill = color)
        
        lineStart = x1 + scale * factor
        lineEnd = x2 - scale * factor

        self.create_line(lineStart, lineEnd, yNew, yNew, self.lineThickness, "black")
  
    def DrawRollerX(self, x, y, scale, factor, color):
        x1 = x + scale * factor
        x2 = x - scale * factor

        yNew = y + scale * factor

        yBottom = yNew + (x1 - x2)/2

        self.canvas.create_polygon(x, y, x1, yNew, x2, yNew, fill = color)
        
        radius = (yBottom - yNew)/2
        centerY = (yBottom + yNew)/2

        centerX1 = x1 - radius
        centerX2 = x2 + radius


        self.create_circle(centerX1, centerY, radius, fill = None)
        self.create_circle(centerX2, centerY, radius, fill = None)

        lineStart = x1 + scale * factor
        lineEnd = x2 - scale * factor

        self.create_line(lineStart, lineEnd, yBottom, yBottom, self.lineThickness, "black")
  
    def DrawRollerY(self, x, y, scale, factor, color):
        
        y1 = y + scale * factor
        y2 = y - scale * factor

        xNew = x + scale * factor

        xBottom = xNew + (y1 - y2)/2

        self.canvas.create_polygon(x, y, xNew, y1, xNew, y2, fill = color)
        
        radius = (xBottom - xNew)/2
        centerX = (xBottom + xNew)/2

        centerY1 = y1 - radius
        centerY2 = y2 + radius


        self.create_circle(centerX, centerY1, radius, fill = None)
        self.create_circle(centerX, centerY2, radius, fill = None)

        lineStart = y1 + scale * factor
        lineEnd = y2 - scale * factor

        self.create_line(xBottom, xBottom, lineStart, lineEnd, self.lineThickness, "black")
    
    def DrawExternalForces(self, force, x, y, color, thickness, scale):
        #self.DrawForceVector(force[0], force[1], x, y, color, thickness, scale)
        self.create_label(x, y + scale, f"{(force[0], force[1])}")
    
    def DrawReactionForces(self, x, y, scale, node):

        reactionX = 0
        reactionY = 0

        if (node.reactions[0] == True):
            reactionX = self.solution[node.XReactionIndex, 0]
        
        if (node.reactions[1] == True):
            reactionY = self.solution[node.YReactionIndex, 0]

        self.create_label(x, y + scale, f"{(int(reactionX), int(reactionY))}")

        #self.DrawForceVector(int(reactionX), int(reactionY), x, y, "black", self.lineThickness, self.scale)

        return
    
    def DrawConnections(self, graph, solution = None):
        Xmin, Xmax, Ymin, Ymax = graph.GetTrussDimensions()

        ## Go Through connections
        for connection in graph.connections:
            nodeA = connection.nodeA
            nodeB = connection.nodeB

            x0 = self.Remap(nodeA.x, Xmin, Xmax, self.offsetX, self.root.winfo_width() - self.offsetX)
            x1 = self.Remap(nodeB.x, Xmin, Xmax, self.offsetX, self.root.winfo_width() - self.offsetX)
            y0 = self.Remap(nodeA.y, Ymin, Ymax, self.root.winfo_height() - self.offsetY, self.offsetY)
            y1 = self.Remap(nodeB.y, Ymin, Ymax, self.root.winfo_height() - self.offsetY, self.offsetY)

            color = "black"
            
            if (type(solution) != type(None)):
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
        #self.root.after(50, self.ReDraw)  
          
class Node():

    def __init__(self, x, y, reactions = (False, False), force = (0, 0)):
        self.id = 0
        self.x = x
        self.y = y
        self.connections = []
        self.reactions = reactions

        self.XReactionIndex = None
        self.YReactionIndex = None

        self.CheckParameters(x, y, reactions, force)

        self.reactionCount = reactions[0] + reactions[1]

        self.forceApplied = False

        if ((force[0] * force[0] + force[1] * force[1]) != 0):
            self.forceApplied = True

        self.force = force

    def CheckParameters(self, x, y, reactions, force):

        if (type(reactions[0]) != bool):
            raise Exception("Reactions should be either True or False!")
        
        if (type(reactions[1]) != bool):
            raise Exception("Reactions should be either True or False!")

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

        #Always have one X, Y and moment balance
        self.equationsCount = 3
    
    def AddNode(self, node):

        node.SetID(self.nodeCount)
        self.nodes.append(node)
        

        self.nodeCount += 1
        self.unknownsCount += node.reactionCount

        #Every node has X, Y balance
        self.equationsCount += 2
         
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

                node.XReactionIndex = reactionEquationIndex

                reactionEquationIndex += 1

            if (node.reactions[1] == 1):
                yReactionEquation[reactionEquationIndex] = 1
                yEquation[reactionEquationIndex] = 1

                torqueReactionEquation[reactionEquationIndex] = node.x - referencePosX


                node.YReactionIndex = reactionEquationIndex
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
    
    def Solve(self):
        self.reactionCount = self.GetReactionCount()
        self.solutionCount = self.connectionCount + self.reactionCount

        self.A, self.b = self.GetMatrix()

        rank = np.linalg.matrix_rank(self.A) 
        
        if (rank > self.unknownsCount):
            raise Exception("SYSTEM OVERDETERMINED! - Too many Force/Moment Balances")

        elif (rank < self.unknownsCount):
            raise Exception("SYSTEM UNDER-DETERMINED! - Not enough Force/Moment Balances")
        
        return self.A, self.b, np.linalg.solve(self.A, self.b)
     
class Solver():
    """
    The primary wrapper class for solving truss problems

    nodeSize: Controls the displayed size of the nodes
    lineThickness: Controls the displayed lineThicknesses
    offset: Controls the canvas offset in pixels (x, y) from the edge of the window
    fontSize: Controls all font sizes
    width: Starting window width
    height: Starting window height
    scale: Scales icons and force vectors

    """
    def __init__(self, nodeSize = 5, lineThickness = 3, offset = (100, 100), fontSize = 20, width = 800, height = 500, scale = 50):
        self.graph = Graph()
        self.window = Window(nodeSize, lineThickness, offset, fontSize, width, height, scale)
        self.solution = None
        self.A = None
        self.b = None

    def AddNodes(self, nodes):
        """
        Add a list of nodes to the solver.
        """
        for node in nodes:
            self.graph.AddNode(node)
    
    def AddConnections(self, connections):
        """
        Adds a list of connections to the solver.
        """
        for connection in connections:
            self.graph.AddConnection(connection)

    def AddConnection(self, connection):
        """
        Adds a single connection to the solver.
        """
        self.graph.AddConnection(connection)

    def AddNode(self, node):
        """
        Adds a single node to the solver.
        """
        self.graph.AddNode(node)

    def Solve(self):
        """
        Solves the truss problem according to assigned nodes and connections.
        """
        self.A, self.b, self.solution = self.graph.Solve()

        return self.solution
    

    def Draw(self):
        """
        Draw truss structure to the window. If the solution has not been calculated it will draw a simple outline.
        """
        self.window.Draw(self.graph, self.solution)
        self.window.root.mainloop()



    
