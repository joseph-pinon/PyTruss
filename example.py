import pyTruss as pt

def GenerateStructure(solver):
    nodeA = pt.Node(0, 0, reactions=(True, True))
    nodeB = pt.Node(0, 3)
    nodeC = pt.Node(3, 0)
    nodeD = pt.Node(3, 6, force = (0, -150))
    nodeE = pt.Node(9, 0)
    nodeF = pt.Node(9, 6, force = (0, -150))
    nodeG = pt.Node(12, 0, reactions = (False, True))
    nodeH = pt.Node(12, 3, force = (-280, 0))

    solver.AddNodes([nodeA, nodeB, nodeC, nodeD, nodeE, nodeF, nodeG, nodeH])

    solver.AddConnection(pt.Connection(nodeA, nodeB))
    solver.AddConnection(pt.Connection(nodeB, nodeC))
    solver.AddConnection(pt.Connection(nodeA, nodeC))
    solver.AddConnection(pt.Connection(nodeB, nodeD))
    solver.AddConnection(pt.Connection(nodeC, nodeD))
    solver.AddConnection(pt.Connection(nodeD, nodeE))
    solver.AddConnection(pt.Connection(nodeC, nodeF))
    solver.AddConnection(pt.Connection(nodeD, nodeF))
    solver.AddConnection(pt.Connection(nodeF, nodeH))
    solver.AddConnection(pt.Connection(nodeH, nodeG))
    solver.AddConnection(pt.Connection(nodeE, nodeH))
    solver.AddConnection(pt.Connection(nodeE, nodeF))
    solver.AddConnection(pt.Connection(nodeE, nodeG))
    
def main():

    solver = pt.Solver()
    GenerateStructure(solver)

    solver.Solve()
    solver.Draw()
    
main()