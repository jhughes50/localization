import networkx
import numpy as np
import torch # pytorch backend
import pygmtools as pygm

class CrossView():
    def __init__(self, config: dict) -> None:
        pass

    def register_graphs(G1: networkx.Graph, G2: networkx.Graph) -> np.array:
        pose = np.eye((4,4))
        n = G1.number_of_nodes()
        m = G2.number_of_nodes()
        K = np.zeros((n*m,n*m))
        for node1, data1 in G1.nodes(data=True):
            for node2, data2 in G2.nodes(data=True):
                if data1['label'] == data2['label']:
                    K[node1, node2] = 1.

        return pose

    def make_compatibility_graph(self) -> None:
        pass

if __name__ == "__main__":
    pass

