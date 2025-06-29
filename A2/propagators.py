#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete problem solution.  

'''
Ausef Yousof - CSC384 A2 Summer 2025
'''

'''This file will contain different constraint propagators to be used within 
   bt_search.

   propagator == a function with the following template
      propagator(csp, newly_instantiated_variable=None)
           ==> returns (True/False, [(Variable, Value), (Variable, Value) ...]

      csp is a CSP object---the propagator can use this to get access
      to the variables and constraints of the problem. The assigned variables
      can be accessed via methods, the values assigned can also be accessed.

      newly_instaniated_variable is an optional argument.
      if newly_instantiated_variable is not None:
          then newly_instantiated_variable is the most
           recently assigned variable of the search.
      else:
          progator is called before any assignments are made
          in which case it must decide what processing to do
           prior to any variables being assigned. SEE BELOW

       The propagator returns True/False and a list of (Variable, Value) pairs.
       Return is False if a deadend has been detected by the propagator.
       in this case bt_search will backtrack
       return is true if we can continue.

      The list of variable values pairs are all of the values
      the propagator pruned (using the variable's prune_value method). 
      bt_search NEEDS to know this in order to correctly restore these 
      values when it undoes a variable assignment.

      NOTE propagator SHOULD NOT prune a value that has already been 
      pruned! Nor should it prune a value twice

      PROPAGATOR called with newly_instantiated_variable = None
      PROCESSING REQUIRED:
        for plain backtracking (where we only check fully instantiated 
        constraints) 
        we do nothing...return true, []

        for forward checking (where we only check constraints with one
        remaining variable)
        we look for unary constraints of the csp (constraints whose scope 
        contains only one variable) and we forward_check these constraints.

        for gac we establish initial GAC by initializing the GAC queue
        with all constaints of the csp


      PROPAGATOR called with newly_instantiated_variable = a variable V
      PROCESSING REQUIRED:
         for plain backtracking we check all constraints with V (see csp method
         get_cons_with_var) that are fully assigned.

         for forward checking we forward check all constraints with V
         that have one unassigned variable left

         for gac we initialize the GAC queue with all constraints containing V.
   '''

def prop_BT(csp, newVar=None):
    '''Do plain backtracking propagation. That is, do no 
    propagation at all. Just check fully instantiated constraints'''
    
    if not newVar:
        return True, []
    for c in csp.get_cons_with_var(newVar):
        if c.get_n_unasgn() == 0:
            vals = []
            vars = c.get_scope()
            for var in vars:
                vals.append(var.get_assigned_value())
            if not c.check(vals):
                return False, []
    return True, []

def prop_FC(csp, newVar=None):
    '''Do forward checking. That is check constraints with 
       only one uninstantiated variable. Remember to keep 
       track of all pruned variable,value pairs and return '''
    #IMPLEMENT

    pruned = []
    dwo = True

    if newVar:
        constraints = csp.get_cons_with_var(newVar)
    else:
        constraints = csp.get_all_cons()
    
    for c in constraints:
        if c.get_n_unasign() == 1:
            dwo, pruned_c = fc_helper(c)
            pruned += pruned_c

            #can stop checking early if DWO for any var, and need to backtrack
            if dwo:
                return False, pruned
    
    return True, pruned     

def prop_GAC(csp, newVar=None):
    '''Do GAC propagation. If newVar is None we do initial GAC enforce 
       processing all constraints. Otherwise we do GAC enforce with
       constraints containing newVar on GAC Queue'''
    #IMPLEMENT

    #dont do this constraint wise in top level f'n because we will maintain
    #a dynamic list of constraints for gac processing
    if newVar:
        return GAC_helper(csp.get_cons_with_var(newVar))
    else:
        return GAC_helper(csp.get_all_cons)
        

'''
HELPERS
'''
def GAC_helper(constraints):
    return True
    


def fc_helper(c):
    pruned = []
    dwo = True

    vals, unasgned, unasgned_idx = assemble_vals_FC(c)

    for dv in unasgned.cur_domain():
        vals[unasgned_idx] = dv
        if c.check(vals):
            dwo = False #we found a support
        else:
            #do pruning
            if unasgned.in_cur_domain(dv):
                unasgned.prune_value(dv)
                pruned.append((unasgned, dv)) #(Variable, value)
        
    return dwo, pruned

def assemble_vals_FC(c):
    vals = []
    i = -1
    unasgned = c.get_unasgn_vars()[0] #FC = only one unassigned var
    for i, var in enumerate(c.get_scope()):
        if var == unasgned:
            #temporary, must be careful to overwrite with test domain values
            vals.append(-1) 
            unasgned_idx = i
        else:
            vals.append(var.get_assigned_value())

    return vals, unasgned, unasgned_idx