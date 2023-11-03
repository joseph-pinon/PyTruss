import pyTruss as pt

def GenerateStructure(solver):
    nodeA = pt.Node(0, 0, reactions=(0, 1))
    nodeB = pt.Node(3, 3)
    nodeC = pt.Node(3, 0, force = (0, -30))
    nodeD = pt.Node(6, 4)
    nodeE = pt.Node(6, 0, force = (0, -30))
    nodeF = pt.Node(9, 3)
    nodeG = pt.Node(9, 0, force = (0, -90))
    nodeH = pt.Node(12, 0, reactions=(1, 1))

    solver.AddNodes([nodeA, nodeB, nodeC, nodeD, nodeE, nodeF, nodeG, nodeH])

    solver.AddConnection(pt.Connection(nodeA, nodeB))
    solver.AddConnection(pt.Connection(nodeB, nodeC))
    solver.AddConnection(pt.Connection(nodeA, nodeC))
    solver.AddConnection(pt.Connection(nodeB, nodeD))
    solver.AddConnection(pt.Connection(nodeC, nodeD))
    solver.AddConnection(pt.Connection(nodeD, nodeE))
    solver.AddConnection(pt.Connection(nodeC, nodeE))
    solver.AddConnection(pt.Connection(nodeD, nodeG))
    solver.AddConnection(pt.Connection(nodeE, nodeG))
    solver.AddConnection(pt.Connection(nodeF, nodeG))
    solver.AddConnection(pt.Connection(nodeD, nodeF))
    solver.AddConnection(pt.Connection(nodeF, nodeH))
    solver.AddConnection(pt.Connection(nodeG, nodeH))
    
def main():

    solver = pt.Solver()
    GenerateStructure(solver)

    solver.Solve()
    solver.Draw()
 
main()