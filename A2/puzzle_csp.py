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

    size = fpuzz_grid[0][0]
    encodings = fpuzz_grid[1:]

    vars = []

    for i in range(size*size):
        vars.append(Variable(name=get_var_name(size, i), domain=list(range(1, size+1))))
    
    csp = CSP(name="binary_ne", vars = vars)
    csp = add_binary_ne_constraints(csp, vars, size)

    return csp, group_by_row(vars, size)
    

def nary_ad_grid(fpuzz_grid):
    ##IMPLEMENT 

    size = fpuzz_grid[0][0]

    vars = [] 
    for i in range(size*size):
        vars.append(Variable(name=get_var_name(size, i), domain=list(range(1, size+1))))

    csp = CSP(name="nary_ne", vars = vars)
    csp = add_nary_ne_constraints(csp, vars, size)

    return csp, group_by_row(vars, size)
    

def caged_csp(fpuzz_grid):
    ##IMPLEMENT 

    csp, var_array = binary_ne_grid(fpuzz_grid)
    csp.name="Caged"
    size = fpuzz_grid[0][0]
    encodings = fpuzz_grid[1:]

    for constraint in get_cage_constraints(csp, encodings, size):
        csp.add_constraint(constraint)

    return csp, var_array
'''
HELPERS
'''

def get_cage_constraints(csp, cages, size):

    constraints = []
    domain = list(range(1, size+1))

    for cage in cages:
        num_of_cells = 0
        affected_vars = []

        if len(cage) == 2: #cell number, value
            #single cell
            cage_con = Constraint(name="="+str(cage[-1]),
                                   scope=[get_var_from_encoding((cage[0]), csp.vars)])
            cage_con.add_satisfying_tuples([(cage[-1],)])

            constraints.append(cage_con)
            continue
        
        operation = cage[-1]
        result = cage[-2]
        
        for i in range(len(cage) - 2):
            num_of_cells += 1
            affected_vars.append(get_var_from_encoding(cage[i], csp.vars))
        
        cage_con = Constraint(name=str(operation) + str(result), scope=affected_vars)

        if (operation == 0):
            satisfying = [p for p in itertools.permutations(domain, num_of_cells) 
                         if sum(p) == result]
            
        elif (operation == 1):
            satisfying = []
            for p in itertools.permutations(domain, num_of_cells):
                if (any_subtract_order_valid(p, result)):
                    satisfying.append(p)

        elif (operation == 2):
            satisfying = []
            for p in itertools.permutations(domain, num_of_cells):
                if (any_divide_order_valid(p, result)):
                    satisfying.append(p)

        elif (operation == 3):
            satisfying = [p for p in itertools.permutations(domain, num_of_cells) 
                         if product_of(p) == result]
        
        cage_con.add_satisfying_tuples(satisfying)
        
        constraints.append(cage_con)
    
    return constraints
            
def any_subtract_order_valid(p, result):
    nums = list(p)

    #flip all (subtraction)
    for num in nums:
        num *= -1
    
    for i in range(len(nums)):
        #flip one (first var)
        nums[i] *= -1
        #sum all (subtract the rest)
        if sum(nums) == result:
            return True
        else:
            #flip back move onto next
            nums[i] *= -1

def any_divide_order_valid(p, result):
    for perm in itertools.permutations(p, len(p)):
        start = perm[0]
        for i in range(len(perm)):
            if i != 0:
                start /= perm[i]
        
        if start == result:
            return True
            

    
def product_of(values):
    ret = 1
    for v in values:
        ret *= v

    return ret
                

def get_var_from_encoding(encoding, vars):

    encoding = str(encoding)

    for var in vars:
        if (str(var.name[0] + var.name[2]) == encoding):
            return var

def get_var_name(size, i):
    '''
    When generating variables specify row_col as var name for easy grouping
    cell 1_1 is top left
    '''
    #this is for iterating over variables in a list (ungrouped)
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
        rows[row-1].append(var)
    return rows

def group_by_col(vars, size):
    '''
    Given a list a variables [v1,v2,v3,v4] group them by row for easy constraint 
    production -> size = 2 -> [[v1,v3],[v2,v4]]
    '''

    cols = [[] for _ in range(size)]
    for var in vars:
        col = int(var.name.split('_')[1])
        cols[col-1].append(var)
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
            new_constraint = Constraint(name=str(combination[0]) + " and " + 
                                        str(combination[1]), scope = combination)
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
