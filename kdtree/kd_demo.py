import copy
import matplotlib.pyplot as plt
import numpy
import pprint
import random
import networkx as nx
import kd_params

def randomizedselect( k, array ):
    if len( array ) == 1:
        return array[0]

    # randomize the pivot to avoid getting stuck in infinite loop
    # if the partition results in array of same length
    pivot = array[ random.randint( 0, len( array ) -1 ) ]
    left, right, match = [], [], []
    for x in array:
        if x < pivot:
            left.append( x )
        elif x == pivot:
            match.append( x )
        else:
            right.append( x )

    size_left, size_match = len( left ), len( match )

    if size_left >= k :
        return randomizedselect( k , left )
    elif k > size_left and k <= size_left + size_match:
        return match[0]
    else:
        return randomizedselect( k - ( size_left + size_match ), right )

def randomized_median( array ):
    n = len( array )
    k = n / 2 + 1 if n % 2 != 0 else n / 2
    return randomizedselect( k, array )

class KDTree( object ):

    _IDX_LOOKUP = { 'x-axis' : 0, 'y-axis' : 1 }

    def __init__( self ):
        self._tree = {}

    def build_kdtree( self, point_list ):

        self._point_list = point_list

        dic = {}
        dim = len( point_list[0] )
        MAXLEVEL = 16

        print 'processing points=%s' % point_list

        def build( d, level, points ):

            if points.size == 0:
                return None
            elif points.size == 1 or level == MAXLEVEL:
                return points

            axis_index = level % dim
            _median = randomized_median( points[ :, axis_index ] )
            median_points = points[ numpy.where( points[ :,axis_index ] == _median ) ]
            left_points = points[ numpy.where( points[ :,axis_index ] < _median ) ]
            right_points = points[ numpy.where( points[ :,axis_index ] > _median ) ]

            d[ 'median_points' ] = median_points
            d[ 'split_axis' ] = 'y-axis' if axis_index else 'x-axis'
            d[ 'left_node' ] = build( {}, level + 1, left_points )
            d[ 'right_node' ] = build( {}, level + 1, right_points )

            return d

        self._tree = build( dic, 0, point_list )

    def pprint( self ):
        pprint.pprint( self._tree )




    def show( self ):
        x, y = zip( *self._point_list )
        plt.autoscale( enable=True, tight=False )
        plt.margins( 0.1, 0.1 )
        plt.plot( x, y, 'o', color='b' )

        yline_color = '#FFB0B0'
        #yline_color = '#FF9595'
        xline_color = '#b0e2ff'
        def plot_separating_planes( v, node):

            idx = KDTree._IDX_LOOKUP[ node[ 'split_axis' ] ]
            pos = node[ 'median_points' ][0][ idx ]

            if idx == 0:
                plt.vlines( pos, v[2], v[3], color=yline_color, linestyle="-.", linewidth=2.0 )
            elif idx == 1:
                plt.hlines( pos, v[0], v[1], color=xline_color, linestyle="-.", linewidth=2.0   )

            if node[ 'left_node' ]:
                v_new = copy.deepcopy( v )
                if idx == 0: # decide to shorten the bounding box along whichever axis               
                    v_new[1] = pos
                elif idx == 1:
                    v_new[3] = pos
                plot_separating_planes( v_new, node[ 'left_node' ] )
            if node[ 'right_node' ]:
                v_new = copy.deepcopy( v )
                if idx == 0: # decide to shorten the bounding box along whichever axis               
                    v_new[0] = pos
                elif idx == 1:
                    v_new[2] = pos
                plot_separating_planes( v_new, node[ 'right_node' ] )

        v_start = numpy.array( plt.axis() )
        plot_separating_planes( v_start, self._tree )

    @staticmethod
    def squared_dist( x, y ):
        v = x - y
        if numpy.vdot( v, v ) == 0:
            return 99999
        else:
            return numpy.vdot( v, v )

    @staticmethod
    def circleOverlap( center, pt, axis, pos ):
        # ( x - a )^2 + ( y - b )^2 = r^2
        #r = numpy.sqrt( numpy.vdot( pt - center ) )
        v = pt - center
        rsqr = numpy.vdot( v, v )
        residue = rsqr - numpy.square( pos - center[ axis ] )
        if residue > 0.0 :
            return True
        return False

    def nearest_neighbour( self, point ):
        # finds the point in the kdtree nearest to the given point

        def np( node ):
            # termination condition
            if node[ 'left_node' ] == None and node[ 'right_node' ] == None:
                return min( [ ( x, KDTree.squared_dist( x, point ) ) for x in node[ 'median_points' ] ], key=lambda x : x[1] )[0]

            candidate = None
            # 1. descend the tree until we get to the first candidate point
            idx = KDTree._IDX_LOOKUP[ node[ 'split_axis' ] ]
            explored = 0
            if point[ idx ] < node[ 'median_points' ][ 0 ][ idx ]:
                if node[ 'left_node' ]:
                    candidate = np( node[ 'left_node' ] )
            else:
                if node[ 'right_node' ]:
                    candidate = np( node[ 'right_node' ] )
                explored = 1

            # select possible candidates from median_points as well
            mp_candidate = min( [ ( x, KDTree.squared_dist( x, point ) ) for x in node[ 'median_points' ] ], key=lambda x : x[1] )[0]

            if candidate is None:
                candidate = mp_candidate
            elif KDTree.squared_dist( point, candidate ) > KDTree.squared_dist( point, mp_candidate ):
                candidate = mp_candidate


            # check if circle centered at point, with radius | point -> candidate | overlap the split plane, if so we need to descend down
            # that node as well
            split_axis = KDTree._IDX_LOOKUP[ node[ 'split_axis' ] ]
            if KDTree.circleOverlap( point, candidate, split_axis, node[ 'median_points' ][0][ split_axis ] ):
                if explored == 0:
                    toexplore = node[ 'right_node' ]
                else:
                    toexplore = node[ 'left_node' ]

                if toexplore:
                    p_candidate = np( toexplore )
                    if KDTree.squared_dist( point, candidate ) > KDTree.squared_dist( point, p_candidate ):
                        candidate = p_candidate

            return candidate

        return np( self._tree )

    def show_nearest_pt( self, pt ):

        n = self.nearest_neighbour( pt )
        print 'nearest point to %s is %s' % ( pt, n )

        v = pt - n
        r = numpy.sqrt( numpy.vdot( v, v ) )
        plt.plot( pt[0], pt[1], 'x', color='r' )
        ax = plt.subplot( 111 )
        ax.add_patch( plt.Circle( pt, r, fill=False ) )

        self.show()

    def add_neighbour_link(self, pt_list, graph):
        for i in xrange(0, len(pt_list)):
            n = self.nearest_neighbour( pt_list[i] )
            graph.add_edge(tuple(n.tolist()), tuple(pt_list[i].tolist()), weight = 1)
            plt.plot([n[0], pt_list[i][0]], 
                     [n[1], pt_list[i][1]],
                     color='y', linestyle=':', linewidth=2)
        self.show()
            
                


def random_points():

    points = []
    for i in xrange( 0, 40 ):
        points.append( ( random.randint( i, 5*i ), random.randint( i, 5*i ) ) )

    return points

"""
   Take input points, remove loops 
   input points is a list of tuples
"""
def preprocess_points_loop(in_points):
    """
        judge wheather two points are close
        x, y are python list
    """
    def is_dist_close (x, y):
        v = numpy.array(x) - numpy.array(y)
        if numpy.vdot( v, v ) <= kd_params.LOOP_REMOVE_DIST:
            return True
        else:
            return False
    def mark_delete(my_list, idx_start, idx_end):
        for i in xrange(idx_start, idx_end+1):
            my_list[i] = 1
    
    remove_index = [0] * len(in_points)
    for i in xrange(0, len(in_points)):
        for j in xrange(i-1, -1, -1):
            if (remove_index[j] == 1): # j is removed
                continue
            else:
                if (is_dist_close(in_points[i], in_points[j]) == True):
                    # remove elements with index [j, i-1]
                    mark_delete(remove_index, j, i-1)
                else:
                    continue
            print remove_index
    
    # output using list comprehension 
    return [item for i,item in enumerate(in_points) if remove_index[i] == 0]     

"""
   Take input points, add intermediate points 
   input points is a list of tuples
"""
def preprocess_points_add_points(in_points):

    
    return in_points
"""
    find the intersection of two line segments. 
    all points are python tuples
"""
def line_intersect(p1_start,p1_end,p2_start,p2_end):
    p = numpy.array(p1_start)
    r = numpy.array(p1_end) - numpy.array(p1_start)
    q = numpy.array(p2_start)
    s = numpy.array(p2_end) - numpy.array(p2_start)
    print r, s
    print numpy.linalg.norm(numpy.cross( r, s ))
    if numpy.cross( r, s ) == 0 and numpy.cross( q-p, r ) == 0:
        t0 = numpy.dot( q-p, r ) / float(numpy.dot( r, r ))
        t1 = t0 + numpy.dot( s, r ) / float(numpy.dot( r, r ))
        if (t0 >= 0 and t0 <= 1) or (t1 >= 0 and t1 <= 1):
            return ["overlap", numpy.cross(p,p)]
        else:
            return ["collinear", numpy.cross(p,p)]
    elif numpy.cross( r, s ) == 0 and numpy.cross( q-p, r ) != 0:
            return ["parallel", numpy.cross(p,p)]
    elif numpy.cross( r, s ) != 0:
        t = numpy.cross( q-p, s ) / numpy.cross( r, s ) 
        u = numpy.cross( q-p, r ) / numpy.cross( r, s ) 
        if ( t >= 0 and t <= 1) and (u >=0 and u <= 1):
            print t, u
            return ["intersect", p+t*r]
        else:
            return ["cross", numpy.cross(p,p)]
    else:
        return ["cross", numpy.cross(p,p)]
            

def test_intersect():
    p =( random.randint( 4, 50 ), random.randint( 4, 50 ) ) 
    r =( random.randint( 4, 50 ), random.randint( 4, 50 ) ) 
    q =( random.randint( 4, 50 ), random.randint( 4, 50 ) ) 
    s =( random.randint( 4, 50 ), random.randint( 4, 50 ) ) 
    p = [float(i) for i in p] 
    r = [float(i) for i in r] 
    q = [float(i) for i in q] 
    s = [float(i) for i in s] 
    print p,r,q,s
    inter_type, inter_point = line_intersect(p,r,q,s)
    if (inter_type == "intersect"):
        print inter_point
        plt.plot([p[0], r[0]], 
                 [p[1], r[1]],
                 color='r', linestyle='-', linewidth=2)
        plt.plot([q[0], s[0]], 
                 [q[1], s[1]],
                 color='r', linestyle='-', linewidth=2)
        plt.plot([0,inter_point[0]],[0, inter_point[1]], 
                 color='b', linestyle='-', linewidth=2)
        plt.show()
        
    

def main():
    #generate graph
    G = nx.Graph()
    #point_list = [(2,3), (5,4), (9,6), (4,7), (8,1), (7,2)]
    #point_list = [2,4,6,8,10,8,6,4,2,0,3,5,7,10,8,6,4,2,1,-2,-5,7]
    point_list = random_points()
    print point_list
    # first: remove loop
    out_list = preprocess_points_loop(point_list)
    print out_list
    # second: add intermediate points if two line segment intersects
    point_list = out_list
    G.add_nodes_from(point_list)
    for i in xrange(0, len(point_list)-1):
        print point_list[i], 
        print "-",
        print point_list[i+1], 
        G.add_edge(point_list[i],point_list[i+1], weight = 1)
        plt.plot([point_list[i][0], point_list[i+1][0]], 
                 [point_list[i][1], point_list[i+1][1]],
                 color='k', linestyle='-', linewidth=2)
    
    array = numpy.array( point_list )
    kdtree = KDTree()
    kdtree.build_kdtree( array )
    #kdtree.pprint()
    kdtree.show()
    #kdtree.show_nearest_pt( numpy.array( point_list[0] ) )
    kdtree.add_neighbour_link( numpy.array( point_list ), G)
    #plt.show()
    #nx.draw(G)
    s_path = nx.astar_path(G, point_list[0], point_list[-1])
    for i in xrange(0, len(s_path)-1):
        print s_path[i], 
        print "-",
        print s_path[i+1], 
        plt.plot([s_path[i][0], s_path[i+1][0]], 
                 [s_path[i][1], s_path[i+1][1]],
                 color='r', linestyle='-', linewidth=2)
    plt.show()

main()
#test_intersect()
