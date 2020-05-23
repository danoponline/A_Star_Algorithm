from queue import PriorityQueue
import math

class Node:
    def __init__(self,intersection_num):
        self.intersection_num =  intersection_num
        self.cumulative_g = None
        self.f = None
        self.parent = None
        self.children = []
        self.index = None
    
    def get_intersection_number(self):
        return self.intersection_num
    
    def add_child(self,child):
        self.children.append(child)
        
    def get_children(self):
        return self.children
    
    def get_parent(self):
        return self.parent
    
    def set_parent(self,node):
        self.parent = node

    def set_cumulative_g(self,g):
        self.cumulative_g = g
        
    def get_cumulative_g(self):
        return self.cumulative_g
    
    def set_f(self,f):
        self.f = f
    
    def get_f(self):
        return self.f
    
    def set_index(self,index):
        self.index = index
        
    def get_index(self):
        return self.index
    
class Tree:
    def __init__(self):
        self.root = None
        
    def set_root(self,node):
        self.root = node
    
    def get_root(self):
        return self.root

# Helper function to calculate straight line distance between two intersections
def cal_displacement(M, int1, int2):
    return math.sqrt((M.intersections[int1][0]-M.intersections[int2][0])**2  + \
    (M.intersections[int1][1]-M.intersections[int2][1])**2)

# Helper function to add children node
def add_children(M,node):
    for intersection in M.roads[node.intersection_num]:
        node.add_child(Node(intersection))

# Helper function to generate a list from the root to specified node
# Only use at the end to generate a solution list
def get_path_list(node):
    path_list = [None for _ in range(node.get_index()+1)]
    while node is not None:
        path_list[node.get_index()] = node.get_intersection_number()
        node = node.get_parent()
    return path_list
    
# Main function for A* Algorithm
def shortest_path(M, start, goal):
    
    # Create start node and populate with its chidren, g, and f
    start_node = Node(start)
    add_children(M,start_node)
    start_node.set_cumulative_g(0)
    start_node.set_f(cal_displacement(M, start, goal))
    start_node.set_index(0)
                     
    # Setup tree as data structure to explore paths                 
    tree = Tree()
    tree.set_root(start_node)
    
    # Setup frontier priority queue to keep track of node 
    # frontier that has the lowest f = g + h
    # Add the start node to the queue
    frontier_queue = PriorityQueue()
    frontier_queue.put((start_node.get_f(),start_node))
    
    # Setup dictionary to keep track of nodes that we have 
    # already visited with the lowest f = g + h
    # Populate with the start point
    visited_dic = {}
    visited_dic[start_node.get_intersection_number()] = start_node.get_f()
    
    # Main loop for A* algorithm
    while frontier_queue.qsize() > 0:
        
        # Pop the node that has the lowest estimated f
        _, parent_node = frontier_queue.get()
        
        # Base case when the solution is found 
        # Goal has the lowest estimated f
        if parent_node.get_intersection_number() == goal:
            return get_path_list(parent_node)
    
        # Expand all children nodes and set cumulative g and f value
        for child_node in parent_node.get_children():
            
            # Set parent and add children for each child node
            child_node.set_parent(parent_node)
            add_children(M,child_node)
            
            # Set g value for each child node
            # Child node g value = cumulative parent g + 
            #                      straigh line distance from parent to child
            child_node.set_cumulative_g(parent_node.get_cumulative_g() +  \
               cal_displacement(M, parent_node.get_intersection_number(), \
                                   child_node.get_intersection_number()   \
                               ))
            
            # Set f value for each child node
            # Child node f value = child node g value+ 
            #                      straigh line distance from child to goal
            child_node.set_f(child_node.get_cumulative_g() +              \
               cal_displacement(M, child_node.get_intersection_number(),  \
                                   goal                                   \
                               ))
            
            # Set index value for each child node (for creating list at the end)
            child_node.set_index(parent_node.get_index() + 1)
            
            # Update visited dictionary and add child to the frontier queue only 
            # when the node is never visited OR the node has a lower estimated 
            # f than previously visited
            if child_node.get_intersection_number() not in visited_dic:
                visited_dic[child_node.get_intersection_number()] = child_node.get_f()
                frontier_queue.put((child_node.get_f(),child_node))
            elif child_node.get_f() < visited_dic[child_node.get_intersection_number()]:
                visited_dic[child_node.get_intersection_number()] = child_node.get_f()
                frontier_queue.put((child_node.get_f(),child_node))
            else:
                pass
    
    # If the queue is empty and the path has never been found, return None   
    return None