#lang dssl2

# Final project: Trip Planner
#
# ** You must work on your own for this assignment. **

# Your program will most likely need a number of data structures, many of
# which you've implemented in previous homeworks.
# We have provided you with compiled versions of homework 3, 4, and 5 solutions.
# You can import them as described in the handout.
# Be sure to extract `project-lib.zip` is the same directory as this file.
# You may also import libraries from the DSSL2 standard library (e.g., cons,
# array, etc.).
# Any other code (e.g., from lectures) you wish to use must be copied to this
# file.

import sbox_hash
import cons
import 'project-lib/graph.rkt'
import 'project-lib/dictionaries.rkt'
import 'project-lib/binheap.rkt'

### Basic Vocabulary Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?
#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

# ListC[T] is a list of `T`s (linear time):
let ListC = Cons.ListC

# List of unspecified element type (constant time):
let List? = Cons.list?

let eight_principles = ["Know your rights.",
                        "Acknowledge your sources.",
                        "Protect your work.",
                        "Avoid suspicion.",
                        "Do your own work.",
                        "Never falsify a record or permit another person to do so.",
                        "Never fabricate data, citations, or experimental results.",
                        "Always tell the truth when discussing your work with your instructor."]
### Input Types ###

#  - a SegmentVector  is VecC[SegmentRecord]
#  - a PointVector    is VecC[PointRecord]
# where
#  - a SegmentRecord  is [Lat?, Lon?, Lat?, Lon?]
#  - a PointRecord    is [Lat?, Lon?, Cat?, Name?]


### Output Types ###

#  - a NearbyList     is ListC[PointRecord]; i.e., one of:
#                       - None
#                       - cons(PointRecord, NearbyList)
#  - a PositionList   is ListC[Position]; i.e., one of:
#                       - None
#                       - cons(Position, PositionList)
# where
#  - a PointRecord    is [Lat?, Lon?, Cat?, Name?]  (as above)
#  - a Position       is [Lat?, Lon?]


# Interface for trip routing and searching:
interface TRIP_PLANNER:
    # Finds the shortest route, if any, from the given source position
    # (latitude and longitude) to the point-of-interest with the given
    # name. (Returns the empty list (`None`) if no path can be found.)
    def find_route(
            self,
            src_lat:  Lat?,     # starting latitude
            src_lon:  Lon?,     # starting longitude
            dst_name: Name?     # name of goal
        )   ->        List?     # path to goal (PositionList)

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position. (Ties for nearest are broken
    # arbitrarily.)
    def find_nearby(
            self,
            src_lat:  Lat?,     # starting latitude
            src_lon:  Lon?,     # starting longitude
            dst_cat:  Cat?,     # point-of-interest category
            n:        nat?      # maximum number of results
        )   ->        List?     # list of nearby POIs (NearbyList)
        
struct pos:
    let lat
    let lon
    
struct POI_struct:
    let cat
    let name

struct pq:
    let node_id
    let distance 
    
struct distance_pred:
    let distance
    let pred
    
class TripPlanner (TRIP_PLANNER):
    let _hash_position
    let _hash_POI
    let _hash_name_to_cat
    let _hash_node_to_POIs
    let _hash_name_to_vector
    let _hash_node_to_pos_vector
    let _graph
    let _node_id
    let _POI_node_id
    
    def __init__(self, road_segments, POI):
        let pos1
        let pos2
        let pos_POI
        let POI_cat_name 
        
        self._node_id = 0
        
        #position -> node_id
        self._hash_position = HashTable(len(road_segments) * 2 + len(POI), make_sbox_hash())
        
        #POI -> node_id
        self._hash_POI = HashTable(len(POI) * 2, make_sbox_hash())
        
        #Name -> category 
        self._hash_name_to_cat = HashTable(len(POI) * 2, make_sbox_hash())
        
        #Node_id -> POI
        self._hash_node_to_POIs = HashTable(len(POI) * 2, make_sbox_hash())
        
        #Name -> Input POI Vector 
        self._hash_name_to_vector = HashTable(len(POI) * 2, make_sbox_hash())
        
        #Node_id -> Input Position [lat, lon] 
        self._hash_node_to_pos_vector = HashTable(len(road_segments) * 2 + len(POI), make_sbox_hash())
        
        self._graph = WuGraph(len(road_segments) * 2 + len(POI))
        
        for x in range(len(road_segments)):
            pos1 = pos(road_segments[x][0], road_segments[x][1])
            pos2 = pos(road_segments[x][2], road_segments[x][3])
           
            if not self._hash_position.mem?(pos1):
                self._hash_position.put(pos1, self._node_id)
                self._hash_node_to_pos_vector.put(self._node_id, [pos1.lat, pos1.lon])
                self._node_id = self._node_id + 1
                
            if not self._hash_position.mem?(pos2):
                self._hash_position.put(pos2, self._node_id)
                self._hash_node_to_pos_vector.put(self._node_id, [pos2.lat, pos2.lon])
                self._node_id = self._node_id + 1
                
            self._graph.set_edge(self._hash_position.get(pos1), self._hash_position.get(pos2), 
                                self.distance(pos1.lat, pos1.lon, pos2.lat, pos2.lon))  
                                   
        for x in range(len(POI)):        
            pos_POI = pos(POI[x][0], POI[x][1])
            POI_cat_name = POI_struct(POI[x][2], POI[x][3])
            
            if not self._hash_position.mem?(pos_POI):
                self._hash_position.put(pos_POI, self._node_id)
                self._hash_POI.put(POI_cat_name, self._node_id)
                self._hash_name_to_cat.put(POI_cat_name.name, POI_cat_name.cat)
                self._hash_node_to_POIs.put(self._node_id, cons(POI_struct(POI[x][2], POI[x][3]), None))
                self._hash_name_to_vector.put(POI_cat_name.name, POI[x])
                self._hash_node_to_pos_vector.put(self._node_id, [pos_POI.lat, pos_POI.lon])
                self._node_id = self._node_id + 1
            else:
                self._hash_POI.put(POI_cat_name, self._hash_position.get(pos_POI))
                self._hash_name_to_cat.put(POI_cat_name.name, POI_cat_name.cat)
                self._POI_node_id = self._hash_position.get(pos_POI)
                self._hash_name_to_vector.put(POI_cat_name.name, POI[x])
                if not self._hash_node_to_POIs.mem?(self._POI_node_id):
                    self._hash_node_to_POIs.put(self._POI_node_id, cons(POI_struct(POI[x][2], POI[x][3]), 
                                                None))
                else:
                    self._hash_node_to_POIs.put(self._POI_node_id, cons(POI_struct(POI[x][2], POI[x][3]), 
                                                self._hash_node_to_POIs.get(self._POI_node_id)))
                                 
    def distance(self, lat1, lon1, lat2, lon2):
        return (((lat2 - lat1)**2 + (lon2 - lon1)**2).sqrt())
        
    def lt?(self, x, y):     
        return x.distance < y.distance
        
    def dijkstras(self, lat, lon):
       let distances = [inf; self._graph.len()]
       let previous = [None; self._graph.len()]
       let done = HashTable(self._graph.len(),make_sbox_hash()) 
       let todo = BinHeap(self._graph.len(), self.lt?)
       let initial_index = self._hash_position.get(pos(lat, lon))
       let smallest
       let holder
        
       distances[initial_index] = 0
       
       #Set every node to false, which
       #means it isn't relaxed
       for x in range(self._graph.len()): 
           done.put(x, False)

       todo.insert(pq(initial_index, 0)) 

       while not todo.len() == 0:
             smallest = todo.find_min().node_id
             todo.remove_min()

             if done.get(smallest) == False:
                 done.put(smallest, True)  
                 for x in Cons.to_vec(self._graph.get_adjacent(smallest)):
                     holder = distances[smallest] + self._graph.get_edge(smallest, x)
                     
                     if holder < distances[x]:
                         distances[x] = holder 
                         previous[x] = smallest
                         todo.insert(pq(x, holder))
       
       return(distance_pred(distances, previous))
        
    def find_route(self, lat, lon, name):  
        if not self._hash_name_to_cat.mem?(name):
            return None
        let path = None
        let start = self._hash_position.get(pos(lat, lon))
        let pred_vec = self.dijkstras(lat, lon).pred
        let cat = self._hash_name_to_cat.get(name)
        let holder = self._hash_POI.get(POI_struct(cat, name))
                 
        while not holder == start:
            path = cons(self._hash_node_to_pos_vector.get(holder), path)
            holder = pred_vec[holder]
    
        path = cons(self._hash_node_to_pos_vector.get(start), path)
        return path
  
    def find_nearby(self, lat, lon, cat, n):
        let category = cat     
        let distances_vec = self.dijkstras(lat, lon).distance
        let todo = BinHeap(self._hash_position.len(), self.lt?)
        let counter = 0
        let node_id
        let POI_list 
        let return_list = None
          
        for x in range(self._hash_position.len()):
            if not distances_vec[x] == inf:
                todo.insert(pq(x, distances_vec[x]))   
            
        while counter < n:
            if todo.len() == 0:
                break
            node_id = todo.find_min().node_id
            todo.remove_min()
            
            if self._hash_node_to_POIs.mem?(node_id):
                POI_list = self._hash_node_to_POIs.get(node_id)
                while not POI_list == None:
                    if POI_list.data.cat == category:
                       return_list = cons(self._hash_name_to_vector.get(POI_list.data.name), return_list)
                       counter = counter + 1
                    POI_list = POI_list.next
                    
        return return_list
             
def example_from_handout():                           
    return TripPlanner([[0,0, 0,1], [0,0, 1,0], [1, 0, 1, 1],
                        [0, 1, 1, 1], [0, 1, 0, 2],
                        [1, 1, 1, 2], [0, 2, 1, 2],
                        [1, 2, 1, 3], [1, 3, -.2, 3.3]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pelmeni"], [0,0, "Food", "Sandwiches"], 
                        [0,1, "Food", "Pasta"], [1, 1, "Bank", "Local Credit Union"], 
                        [-.2, 3.3, "Food", "Burritos"]])                                                     
test 'Example test':
    assert example_from_handout().find_route(0, 0, "Sandwiches") == \
        cons([0, 0], None)
    assert example_from_handout().find_route(0, 1, "Sandwiches") == \
        cons([0, 1], cons ([0, 0], None))
    assert example_from_handout().find_route(0, 0, "Sushi") == \
        None      
    assert example_from_handout().find_route(1, 1, "Sandwiches") == \
        cons([1,1], cons([0, 1], cons ([0, 0], None)))
        
def my_first_example():
   return TripPlanner([[0,0, 0,1], [0,0, 1,0], [1, 0, 1, 1]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pelmeni"], 
                        [0,0, "JACK", "O'KEEFE"]])

test 'My first find_route test':
  assert my_first_example().find_route(0, 0, "Pelmeni") == \
         cons([0,0], cons([0,1], None))

test 'My first find_nearby test':
    assert my_first_example().find_nearby(0, 0, "food", 1) == \
        cons([0,1, "food", "Pelmeni"], None)
        
test 'My second find_nearby test':
    assert my_first_example().find_nearby(0, 0, "Burgers", 1) == \
        None
        
let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])

test 'test1':
    assert tp.find_nearby(0, 0, 'food', 1) \
      == None
#I had more tests, but deleted them as I couldn't formate them right, so I visually inspected
#the results 