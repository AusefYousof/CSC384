#Look for #IMPLEMENT tags in this file.
'''
All encodings need to return a CSP object, and a list of lists of Variable objects 
representing the board. The returned list of lists is used to access the 
solution. 

For example, after these three lines of code

    csp, var_array = caged_csp(board)
    solver = BT(csp)
    solver.bt_search(prop_FC, var_ord)

var_array[0][0].get_assigned_value() should be the correct value in the top left
cell of the FunPuzz puzzle.

The grid-only encodings do not need to encode the cage constraints.

1. binary_ne_grid (worth 10/100 marks)
    - An enconding of a FunPuzz grid (without cage constraints) built using only 
      binary not-equal constraints for both the row and column constraints.

2. nary_ad_grid (worth 10/100 marks)
    - An enconding of a FunPuzz grid (without cage constraints) built using only n-ary 
      all-different constraints for both the row and column constraints. 

3. caged_csp (worth 25/100 marks) 
    - An enconding built using your choice of (1) binary binary not-equal, or (2) 
      n-ary all-different constraints for the grid.
    - Together with FunPuzz cage constraints.

'''
from cspbase import *
import itertools
from itertools import combinations

#fpuzz_grid = [[3], [11,12,13,6,0], [21,22,12,7,2], ....]
#return csp object, [list of variables]

def binary_ne_grid(fpuzz_grid):
    ##IMPLEMENT

    #Binary not equal constraints on variables for row and column
    #Two vars in the same row cannot be equal, same col cannot be equal

    size = fpuzz_grid[0][0]
    encodings = fpuzz_grid[1:]

    vars = []

    for i in range(len(size*size)):
        vars.append(Variable(name=get_var_name(size, i)), domain=list(range(1, size+1)))
    
    csp = CSP(name="binary_ne", vars = vars)
    csp = add_binary_ne_constraints(csp, vars, size)

    return csp, group_by_row(vars, size)
    

def nary_ad_grid(fpuzz_grid):
    
    size = fpuzz_grid[0][0]

    vars = [] 
    for i in range(len(size*size)):
        vars.append(Variable(name=get_var_name(size, i)), domain=list(range(1, size+1)))

    csp = CSP(name="nary_ne", vars = vars)
    csp = add_nary_ne_constraints(csp, vars, size)

    return csp, group_by_row(vars, size)
    

def caged_csp(fpuzz_grid):
    ##IMPLEMENT 

    return True

'''
HELPERS
'''

def get_var_name(size, i):
    '''
    When generating variables specify row_col as var name for easy grouping
    cell 1_1 is top left
    '''
    row = (i // size) + 1
    col = (i % size) + 1
    return f"{row}_{col}"

def group_by_row(vars, size):
    '''
    Given a list a variables [v1,v2,v3,v4] group them by row for easy constraint 
    production -> size = 2 -> [[v1,v2],[v3,v4]]
    '''
    rows = [[] for _ in range(size)]
    for var in vars:
        row = int(var.name.split('_')[0])
        rows[row].append(var)
    return rows

def group_by_col(vars, size):
    '''
    Given a list a variables [v1,v2,v3,v4] group them by row for easy constraint 
    production -> size = 2 -> [[v1,v3],[v2,v4]]
    '''

    cols = [[] for _ in range(size)]
    for var in vars:
        col = int(var.name.split('_')[1])
        cols[col].append(var)
    return cols

def add_binary_ne_constraints(csp, vars, size):

    '''
    Add generated binary-not-equal constraints to constraint list which is added 
    to an updated csp during binary_ne_grid
    '''

    constraints = []
    rows = group_by_row(vars, size)
    cols = group_by_col(vars, size)

    domain = list(range(1, size + 1))
    
    for row in rows:
        constraints += init_binary_consts(row, domain)

    for col in cols:
        constraints += init_binary_consts(col, domain)

    for c in constraints:
        csp.add_constraint(c)

    return csp
            
def init_binary_consts(row_or_col, domain):
    '''
    Generate binary-not-equal constraints for all variables across each row and col,
    generated all possible valid combinations of values for two variables s.t X != Y
    '''

    constraints = []

    for combination in list(combinations(row_or_col, 2)):
            new_constraint = Constraint(name=combination[0] + " and " + 
                                        combination[1], scope = combination)
            new_constraint.add_satisfying_tuples([(i, j) for i in domain for j in domain if i != j])
            constraints.append(new_constraint)
    
    return constraints

def add_nary_ne_constraints(csp, vars, size):
    '''
    Add the generated nary constraints to csp
    '''

    constraints = []
    rows = group_by_row(vars, size)
    cols = group_by_col(vars, size)

    domain = list(range(1, size + 1))

    for row in rows:
        constraints += init_nary_consts(row, domain, size)

    for col in cols:
        constraints += init_nary_consts(col, domain, size)

    for c in constraints:
        csp.add_constraint(c)

    return csp


def init_nary_consts(row_or_col, domain, size):
    '''
    Generate nary constraints, that is, for a given row or column, generate
    all possible values of V1, V2... s.t none are equal to each other
    '''

    new_constraint = Constraint(name="nary", scope = row_or_col)
    for pm in itertools.permutations(domain, size):
        new_constraint.add_satisfying_tuples(tuple(pm))

    return [new_constraint]
