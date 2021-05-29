#! /usr/bin/python3
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
import sys
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar


from SearchAlgorithms import *


 

class PrettyWidget(QWidget):

    NumButtons = ['BFS','DFS', 'UCS','Astar',"GBFS", "Clear","Clean"]

    def __init__(self):


        super(PrettyWidget, self).__init__()        
        font = QFont()
        font.setPointSize(16)        
        self.initUI()

####################################################### UI intialization function ##################################################################
    def initUI(self):
        
        self.setGeometry(100,100, 1400, 1050)
        self.center()
        self.setWindowTitle('Graph Search Algorithms')

        grid = QGridLayout()
        self.setLayout(grid)
        self.createVerticalGroupBox()
        self.createTestGroupBox()


        ### search algorithms layout buttons ###
        buttonLayout = QVBoxLayout()
        buttonLayout.addWidget(self.verticalGroupBox)
        
        
        ### add nodes buttons and textbox layout ###
        testLayout1= QVBoxLayout()
        testLayout1.addWidget(self.testGroupBox1)
        
        ### add edges buttons and textbox layout ###
        testLayout3= QVBoxLayout()
        testLayout3.addWidget(self.testGroupBox3)
        
        ### directed and undirected graph buttons layout ###
        testLayout4= QVBoxLayout()
        testLayout4.addWidget(self.testGroupBox4)
        
        ### draw graph button layout ###
        testLayout5= QVBoxLayout()
        testLayout5.addWidget(self.testGroupBox5)
        
        
        ### heuristics buttons and textbox layout ###
        testLayout6= QVBoxLayout()
        testLayout6.addWidget(self.testGroupBox6)
        
        ### start and goal nodes layout ###
        testLayout7= QVBoxLayout()
        testLayout7.addWidget(self.testGroupBox7)
        
        
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure) 
        
        grid.addWidget(self.canvas, 0, 0,7,9)          
        grid.addLayout(buttonLayout, 0, 10,2,2)
        grid.addLayout(testLayout5,7,10,1,2)
        grid.addLayout(testLayout4,2,10,1,2)
        grid.addLayout(testLayout1,4,10,1,2)
        grid.addLayout(testLayout3,6,10,1,2)
        grid.addLayout(testLayout6,5,10,1,2)
        grid.addLayout(testLayout7,3,10,1,2)
        self.show()
        
######################################################### LAYOUT ##############################################

    def createVerticalGroupBox(self):
        self.verticalGroupBox = QGroupBox()

        layout = QVBoxLayout()
        for i in  self.NumButtons:
                button = QPushButton(i)
                button.setObjectName(i)
                layout.addWidget(button)
                layout.setSpacing(10)
                self.verticalGroupBox.setLayout(layout)
                button.clicked.connect(self.submitCommand)
    

    
    def createTestGroupBox(self):
        self.testGroupBox1=QGroupBox()
        self.testGroupBox3=QGroupBox()
        self.testGroupBox4=QGroupBox()
        self.testGroupBox5=QGroupBox()
        self.testGroupBox6=QGroupBox()
        self.testGroupBox7=QGroupBox()
        
        self.layout1=QHBoxLayout()
        self.layout3=QHBoxLayout()
        self.layout4=QHBoxLayout()
        self.layout5=QHBoxLayout()
        self.layout6=QHBoxLayout()
        self.layout7=QHBoxLayout()
        

        self.text1=QLineEdit()
        self.text1.setObjectName("Node1Val")
        self.text1.setMaximumWidth(50)
        self.layout1.addWidget(self.text1)
        
        self.text2=QLineEdit()
        self.text2.setObjectName("Node2Val")
        self.text2.setMaximumWidth(50)
        self.layout1.addWidget(self.text2)

        button1=QPushButton("Add Nodes")
        button1.setObjectName("Nodes")
        self.layout1.addWidget(button1)
        button1.clicked.connect(self.submitCommand)
        self.testGroupBox1.setLayout(self.layout1)
        

        
        self.text3=QLineEdit()
        self.text3.setObjectName("Edge")
        self.text3.setMaximumWidth(50)
        self.layout3.addWidget(self.text3)

        button3=QPushButton("Edge Weight from Node1 -> Node2")
        button3.setObjectName("Edge")
        self.layout3.addWidget(button3)
        button3.clicked.connect(self.submitCommand)
        self.testGroupBox3.setLayout(self.layout3)
                
        button4=QPushButton("Directed Graph")
        button4.setObjectName("Directed")
        self.layout4.addWidget(button4)
        button4.clicked.connect(self.submitCommand)
                                     
        button5=QPushButton("Undirected Graph")
        button5.setObjectName("Undirected")
        self.layout4.addWidget(button5)
        button5.clicked.connect(self.submitCommand)
        self.testGroupBox4.setLayout(self.layout4)
        
        self.button6=QPushButton("Draw Graph")
        self.button6.setObjectName("Draw")
        self.layout5.addWidget(self.button6)
        self.button6.clicked.connect(self.submitCommand)
        self.testGroupBox5.setLayout(self.layout5)
        
        self.text4=QLineEdit()
        self.text4.setObjectName("heuristic1text")
        self.text4.setMaximumWidth(50)
        self.layout6.addWidget(self.text4)
        
        button7=QPushButton("heuristic Node 1")
        button7.setObjectName("heuristic1")
        self.layout6.addWidget(button7)
        button7.clicked.connect(self.submitCommand)
        self.testGroupBox6.setLayout(self.layout6)
        
        self.text7=QLineEdit()
        self.text7.setObjectName("heuristic2text")
        self.text7.setMaximumWidth(50)
        self.layout6.addWidget(self.text7)       


        
        button8=QPushButton("heuristic Node 2")
        button8.setObjectName("heuristic2")
        self.layout6.addWidget(button8)
        button8.clicked.connect(self.submitCommand)
        self.testGroupBox6.setLayout(self.layout6)
        
        self.text5=QLineEdit()
        self.text5.setObjectName("Start")
        self.text5.setMaximumWidth(50)
        self.layout7.addWidget(self.text5)
        
        self.text6=QLineEdit()
        self.text6.setObjectName("Goal")
        self.text6.setMaximumWidth(50)
        self.layout7.addWidget(self.text6)

        button8=QPushButton("Enter Start and Goal Nodes")
        button8.setObjectName("Search")
        self.layout7.addWidget(button8)
        button8.clicked.connect(self.submitCommand)
        self.testGroupBox7.setLayout(self.layout7)
        
        
        
    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
        
######################################################### Auxiliray functions ############################################0      
    
    def heur(self,p1,p2):
        return self.node1H[p1]
        

    def submitCommand(self):
        eval('self.' + str(self.sender().objectName()) + '()')

    def Search(self):
        start=self.text5.text()
        goal = self.text6.text()
        self.Goal=goal
        self.Start=start
        
        self.text5.clear()
        self.text6.clear()

    def heuristic1(self):
        self.h=int(self.text4.text())
        self.node1H[self.NodeA]=self.h
        self.text4.clear()
        
    def heuristic2(self):
        self.h2=int(self.text7.text())
        self.node1H[self.NodeB]=self.h2
        self.text7.clear()
    
    def Clean(self):
        self.figure.clf()
        self.G.clear()
        self.canvas.update()
        self.canvas.draw_idle()
    def Clear(self):
        self.figure.clf()
        self.canvas.update()
        self.canvas.draw_idle()


    def Nodes(self):
        print("Nodes pressed")

        currNode = self.text1.text()
        self.G.add_node(currNode)
        self.NodeA = currNode
        self.text1.clear()

        currNode2 = self.text2.text()
        self.G.add_node(currNode2)
        self.NodeB = currNode2
        self.text2.clear()

    def Edge(self):
        self.figure.clf()
        print("Edge pressed")
        print(self.text3.text())
        if self.text3.text():
            self.weight = int(self.text3.text())
            self.G.add_edge(self.NodeA, self.NodeB,weight=self.weight, length=5)
        else:
            self.G.add_edge(self.NodeA, self.NodeB, length=5)

        self.text3.clear()

    def Directed(self):
        self.G = nx.DiGraph()
        self.node1H = {}
        self.node2H = {}
        self.visited = set()
        self.result = []

    def Undirected(self):
        self.G = nx.Graph()
        self.node1H = {}
        self.node2H = {}
        self.visited = set()
        self.result = []

    def Draw(self):    
        print("Draw pressed")
        self.figure.clf()

        self.node_pos = nx.circular_layout(self.G)
        arc_weight = nx.get_edge_attributes(self.G, 'weight')
        nx.draw_networkx(self.G, self.node_pos, node_size=1000)
        nx.draw_networkx_edges(self.G, self.node_pos, width=4,arrowsize=30)
        nx.draw_networkx_edge_labels(self.G, self.node_pos, edge_labels=arc_weight)
        plt.axis('off')

        #print("h1",self.node1H)
        self.canvas.draw_idle()

        

############################################################# SEARCH ALGORITHMS ############################################################    

    def BFS(self):
        self.figure.clf()

        self.bfspath=bfs(self.G,self.Start,self.Goal)
        self.sp=self.bfspath 
        
        arc_weight = nx.get_edge_attributes(self.G, 'weight')
        red_edges = list(zip(self.sp, self.sp[1:]))
        self.node_col = ['red' if not node in self.sp else 'green' for node in self.G.nodes()]
        self.edge_col = ['black' if not edge in red_edges else 'green' for edge in self.G.edges()]
        
        nx.draw_networkx(self.G, self.node_pos,node_color=self.node_col,node_size=1000)
        nx.draw_networkx_edges(self.G, self.node_pos,edge_color=self.edge_col,width=6,arrowsize=30)
        nx.draw_networkx_edge_labels(self.G, self.node_pos, edge_labels=arc_weight)
  
        plt.axis('off')


        self.canvas.draw_idle()

    def DFS(self):
        self.figure.clf()

        self.dfslist=dfs(self.G,self.Start,self.Goal)
        self.dfspath=list(self.dfslist[0][0:])
        self.sp = self.dfspath 
        
        arc_weight = nx.get_edge_attributes(self.G, 'weight')
        red_edges = list(zip(self.sp, self.sp[1:]))
        
        self.node_col = ['red' if not node in self.sp else 'green' for node in self.G.nodes()]
        self.edge_col = ['black' if not edge in red_edges else 'green' for edge in self.G.edges()]
        
        nx.draw_networkx(self.G, self.node_pos,node_color=self.node_col,node_size=1000)
        nx.draw_networkx_edges(self.G, self.node_pos,edge_color=self.edge_col,width=6,arrowsize=30)
        nx.draw_networkx_edge_labels(self.G, self.node_pos, edge_labels=arc_weight)
        plt.axis('off')
       
        self.canvas.draw_idle()

    def UCS(self):
        self.figure.clf()

        self.ucspath=ucs(self.G,self.Start,self.Goal)
        self.sp=self.ucspath 
        
        arc_weight = nx.get_edge_attributes(self.G, 'weight')
        red_edges = list(zip(self.sp, self.sp[1:]))
        
        self.node_col = ['red' if not node in self.sp else 'green' for node in self.G.nodes()]
        self.edge_col = ['black' if not edge in red_edges else 'green' for edge in self.G.edges()]
        
        nx.draw_networkx(self.G, self.node_pos,node_color=self.node_col,node_size=1000)
        nx.draw_networkx_edges(self.G, self.node_pos,edge_color=self.edge_col,width=6,arrowsize=30)
        nx.draw_networkx_edge_labels(self.G, self.node_pos, edge_labels=arc_weight)
        plt.axis('off')
       
        self.canvas.draw_idle()

    def Astar(self):
        self.figure.clf()

        self.astarpath=astar(self.G,self.Start,self.Goal, heuristic=self.heur, weight=self.weight)
        self.sp=self.astarpath 
        
        arc_weight = nx.get_edge_attributes(self.G, 'weight')
        self.red_edges = list(zip(self.sp, self.sp[1:]))
        
        self.node_col = ['red' if not node in self.sp else 'green' for node in self.G.nodes()]
        self.edge_col = ['black' if not edge in self.red_edges else 'green' for edge in self.G.edges()]
        
        nx.draw_networkx(self.G, self.node_pos,node_color=self.node_col,node_size=1000)
        nx.draw_networkx_edges(self.G, self.node_pos,edge_color=self.edge_col,width=6, arrowsize=30)
        nx.draw_networkx_edge_labels(self.G, self.node_pos, edge_labels=arc_weight)
        plt.axis('off')
        self.canvas.draw_idle()

    def GBFS(self):
        self.figure.clf()
        
        self.gbfspath=GBFS(self.G,self.Start,self.Goal, heuristic=self.heur)
        self.sp=self.gbfspath 
        
        arc_weight = nx.get_edge_attributes(self.G, 'weight')
        self.red_edges = list(zip(self.sp, self.sp[1:]))
        
        self.node_col = ['red' if not node in self.sp else 'green' for node in self.G.nodes()]
        self.edge_col = ['black' if not edge in self.red_edges else 'green' for edge in self.G.edges()]
        
        nx.draw_networkx(self.G, self.node_pos,node_color=self.node_col,node_size=1000)
        nx.draw_networkx_edges(self.G, self.node_pos,edge_color=self.edge_col,width=6,arrowsize=30)
        nx.draw_networkx_edge_labels(self.G, self.node_pos, edge_labels=arc_weight)
        plt.axis('off')

        self.canvas.draw_idle()







if __name__ == '__main__':
    Result = []
    visited = set()
    app = QApplication(sys.argv)
    app.aboutToQuit.connect(app.deleteLater)
    app.setStyle(QStyleFactory.create("gtk"))
    screen = PrettyWidget() 
    screen.show()   
    sys.exit(app.exec_())
