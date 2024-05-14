from factest_base_z3 import FACTEST_Z3

import spot
spot.setup()
import z3
import numpy as np
import polytope as pc
import copy

import random

####################
# Modified LTL2GBA #
####################
class buchi_from_ltl:
    def __init__(self, ltl_formula, env) -> None:
        self.ltl_str = ltl_formula
        self.env = env

        dim_mat = self.env[list(self.env.keys())[0]].A
        self.dims = len(dim_mat[0])

        self.getAlphabet()
        self.buchi_states, self.buchi_inits, self.buchi_AP, self.buchi_alphabet, self.buchi_transitions, self.buchi_acceptances, self.buchi_run = self.getBuchi()

        self.getAllTransitions()
        
    def getAlphabet(self):
        AP = list(self.env.keys())
        set_size = len(AP)
        power_set_size = 2**(set_size)
        
        self.alphabet = []

        # Code from: https://www.geeksforgeeks.org/power-set/
        for counter in range(0, power_set_size): 
            new_letter = []
            for j in range(0, set_size): 
                if((counter & (1 << j)) > 0): 
                    new_letter.append(AP[j])
                    
            self.alphabet.append(new_letter)
    
    def getBuchiAlphabet(self, AP_list):
        set_size = len(AP_list)
        power_set_size = 2**(set_size)
        
        buchi_alphabet = []

        # Code from: https://www.geeksforgeeks.org/power-set/
        for counter in range(0, power_set_size): 
            new_letter = []
            for j in range(0, set_size): 
                if((counter & (1 << j)) > 0): 
                    new_letter.append(AP_list[j])
                   
            buchi_alphabet.append(new_letter)
        
        return buchi_alphabet

    def checkIntersection(self, letter):
        #Code from: https://github.com/IllinoisReliableAutonomyGroup/topology-feasibility/blob/main/intersection.py 
        x = [z3.Real(f'x_{i}') for i in range(self.dims)]
        s = z3.Solver()

        for key in letter:
            env_poly = self.env[key]
            A = env_poly.A
            b = env_poly.b

            for i in range(len(A)):
                s.add(z3.Sum([A[i][j] * x[j] for j in range(self.dims)]) <= b[i])

        return s.check() == z3.sat

    def getExclusion(self):
        exclusion_str = 'true'
        AP_set = list(self.env.keys())
        for letter in self.alphabet:
            if letter == []:
                new_exclusion_str = ''
                for AP in AP_set:
                    if new_exclusion_str == '':
                        new_exclusion_str = '!'+str(AP)
                    else:
                        new_exclusion_str = new_exclusion_str + ' & !'+str(AP)
                exclusion_str = exclusion_str + ' & !('+new_exclusion_str+')'

            if not self.checkIntersection(letter):
                new_exclusion_str = ''
                for AP in AP_set:
                    if AP in letter:
                        if new_exclusion_str == '':
                            new_exclusion_str = AP
                        else:
                            new_exclusion_str = new_exclusion_str + ' & '+AP
                    else:
                        if new_exclusion_str == '':
                            new_exclusion_str = '!'+AP
                        else:
                            new_exclusion_str = new_exclusion_str + ' & !'+AP
                exclusion_str = exclusion_str + ' & !('+new_exclusion_str+')'
        return exclusion_str
        
    def getBuchi(self):
        exclusion_str = self.getExclusion()
        new_ltl_str = self.ltl_str+' & G ('+exclusion_str+')'
        
        a = spot.translate(new_ltl_str, 't')
        aut_string = a.to_str('hoa')

        buchi_states = []
        buchi_AP = []
        buchi_inits = []
        buchi_transitions = {}
        buchi_acceptances = {}

        for line in aut_string.splitlines():
            # Get initial states for automaton
            if 'Start:' in line:
                start_init_state = False
                init_state = ''
                for s in line:
                    if start_init_state:
                        init_state = init_state + s
                    if s == ' ':
                        start_init_state = True
                buchi_inits.append(init_state)

            # Get AP and the order used in TBA
            if 'AP:' in line:
                start_AP = False
                new_AP = ''
                for s in line:
                    if s == '"':
                        start_AP = not start_AP
                        if not start_AP:
                            buchi_AP.append(new_AP)
                            new_AP = ''
                    
                    if start_AP and s != '"':
                        new_AP = new_AP + s

            # Get accpetance conditions for the automata
            if 'Acceptance:' in line:
                start_acceptance = False
                num_acceptances = ''
                for s in line:
                    if s == ' ':
                        start_acceptance = not start_acceptance
                    if start_acceptance:
                        num_acceptances = num_acceptances + s
                for i in range(int(num_acceptances)):
                    buchi_acceptances[str(i)] = []
            
            # Get states used in TBA
            if 'State:' in line:
                start_state = False
                new_state = ''
                for s in line:
                    if start_state:
                        new_state = new_state + s

                    if s == ' ':
                        start_state = not start_state
                buchi_states.append(new_state)
                
                curr_state = new_state
                buchi_transitions[curr_state] = []

            # Get the transition function for the automata
            if '[' in line:
                end_transition = False
                transition = ''
                start_new_state = False
                new_state = ''
                start_acceptance_condition = False
                new_acceptance = '' #TODO: Need to update acceptance conditions
                acceptance_ids = []

                for s in line:
                    if not end_transition:
                        transition = transition + s
                        if s == ']':
                            end_transition = True
                            start_new_state = not start_new_state

                    elif start_new_state:
                        if s != ' ':
                            if s != '{':
                                new_state = new_state + s
                            else:
                                start_new_state = not start_new_state
                                start_acceptance_condition = True
                    
                    elif start_acceptance_condition:
                        if s != '}':
                            new_acceptance = new_acceptance + s 
                        else:
                            pass
                            
                letter = self.transitionToLetter(transition, buchi_AP)
                buchi_transitions[curr_state].append((letter, new_state))
                for id in acceptance_ids:
                    buchi_acceptances[id].append((curr_state, letter, new_state))
             
        buchi_alphabet = self.getBuchiAlphabet(buchi_AP)

        aut_run = a.accepting_run() #TODO: WE CAN CHANGE THIS TO ACCEPTING WORD FROM SPOT
        print('accepting run', a.accepting_word())
        
        prefix_run = []
        for i in range(len(aut_run.prefix)):
            letter = str(spot.bdd_format_formula(a.get_dict(), aut_run.prefix[i].label))
            new_letter = self.spotLetterToLetter(letter, buchi_AP)
            prefix_run.append(new_letter)

        cycle_run = []
        for i in range(len(aut_run.cycle)):
            letter = str(spot.bdd_format_formula(a.get_dict(), aut_run.cycle[i].label))
            new_letter = self.spotLetterToLetter(letter, buchi_AP)
            cycle_run.append(new_letter)

        buchi_run = {'prefix': prefix_run, 'cycle':cycle_run}
        
        return buchi_states, buchi_inits, buchi_AP, buchi_alphabet, buchi_transitions, buchi_acceptances, buchi_run

    def spotLetterToLetter(self, letter, AP_list):
        stop_letter = False
        curr_AP = ''
        no_AP = []
        yes_AP = []
        for s in letter:
            if s == ' ':
                stop_letter = not stop_letter
                if stop_letter:
                    if '!' in curr_AP:
                        no_AP.append(curr_AP[1:])
                        curr_AP = ''
                    else:
                        yes_AP.append(curr_AP)
                        curr_AP = ''

            if not stop_letter and s!=' ':
                curr_AP = curr_AP+s 
        if '!' in curr_AP:
            no_AP.append(curr_AP[1:])
            curr_AP = ''
        else:
            yes_AP.append(curr_AP)
            curr_AP = ''
        
        new_letter = []
        for ap in AP_list:
            if ap not in no_AP:
                new_letter.append(ap)

        return new_letter

    def transitionToLetter(self, transition, AP_list):
        yes_keys = []
        no_keys = []
        all_keys = []

        curr_key = ''
        for i in range(len(transition)):
            s = transition[i]
            if s != '&' and s != ' ' and s != '['  and s != ']' and s != '|':
                curr_key = curr_key + s
            else:
                if curr_key != '':
                    all_keys.append(curr_key)
                    curr_key = ''

        for key in all_keys:
            if '!' in key:
                no_keys.append(int(key[1:]))
            else:
                yes_keys.append(int(key))

        for i in range(len(list(self.env.keys()))):
            if i not in no_keys and i not in yes_keys:
                yes_keys.append(i)
        
        yes_keys.sort()
        no_keys.sort()

        letter = []
        for key in yes_keys:
            letter.append(AP_list[key])

        return letter

    def getAllTransitions(self):
        self.transitions_in_dict = {}
        self.transitions_out_dict = {}

        for state_key in self.buchi_transitions.keys():

            if state_key not in self.transitions_out_dict.keys():
                self.transitions_out_dict = {state_key : []}

            for transition in self.buchi_transitions[state_key]:
                if transition[1] not in self.transitions_in_dict.keys():
                    self.transitions_in_dict = {transition[1] : []}

                self.transitions_out_dict[state_key].append(transition[0])
                self.transitions_in_dict[transition[1]].append(transition[0])
        
        self.all_transitions = []
        for state_key in self.transitions_out_dict.keys():
            for q0 in self.transitions_in_dict[state_key]:
                for q1 in self.transitions_out_dict[state_key]:
                    if q0 != q1:
                        self.all_transitions.append((q0,q1))

###############################
# The main omega-FACTEST loop #
###############################

class omega_FACTEST(buchi_from_ltl):
    def __init__(self, ltl_formula, env, model = None, workspace = None, seg_max = 3, part_max = 1, shrinking_constant = 0.1, max_shrinking_depth = 5, print_statements = False) -> None:
        super().__init__(ltl_formula, env)

        self.workspace = workspace
        self.model = model

        AP_list = []
        for transition in self.all_transitions:
            AP_list.append(transition[0][0])
            AP_list.append(transition[1][0])

        self.seg_max = seg_max
        self.part_max = part_max
        self.shrinking_constant = shrinking_constant
        self.max_shrinking_depth = max_shrinking_depth
        self.print_statements = print_statements

        self.flow_cache = {}
        self.init_sets = copy.deepcopy(env)
        self.terminal_sets = copy.deepcopy(env)

        key_list = list(self.init_sets.keys())

        for init_key in key_list:
            if init_key not in AP_list:
                del self.init_sets[init_key]
                del self.terminal_sets[init_key]

        self.runOmega()
    
    def runOmega(self):
        converged = False
        i = 1
        while not converged:
            converged = True
            for goal_key in self.terminal_sets.keys():
                self.terminal_sets[goal_key] = []

            for transition in self.all_transitions:
                init_key = transition[0][0]
                goal_key = transition[1][0]
                unsafe_keys = list(self.env.keys())
                unsafe_keys.remove(init_key)
                unsafe_keys.remove(goal_key)

                initial_poly = self.init_sets[init_key]
                goal_poly = self.init_sets[goal_key]
                avoid = [self.env[avoid_key] for avoid_key in unsafe_keys]
                
                factest = FACTEST_Z3(initial_poly, goal_poly, avoid, model=self.model, workspace=self.workspace, seg_max=self.seg_max, part_max=self.part_max, print_statements=self.print_statements) #TODO: NEED TO ADD IN THE MODEL STUFF
                final_parts = factest.run()
                del factest

                for partition_key in final_parts.keys():
                    partition_poly = final_parts[partition_key]['poly']
                    partition_xref = final_parts[partition_key]['xref']

                    if partition_xref != None:
                        N = len(partition_xref) - 1
                        last_err = self.model.errBound(partition_poly, N)
                        self.terminal_sets[goal_key].append((partition_xref, last_err))

                    if partition_xref == None:
                        print("No solution from current",init_key,"to current",goal_key)
                        new_poly = self.shrinkPolytope(initial_poly)
                        if type(new_poly) == None: # or i >= self.max_shrinking_depth:
                            print('Solution cannot be found!')
                            return None
                        else:
                            self.init_sets[init_key] = new_poly
                            if i < self.max_shrinking_depth:
                                converged = False
                        break  
                    
                if (init_key+','+goal_key) not in self.flow_cache.keys():
                    self.flow_cache[init_key+','+goal_key] = final_parts
                else:
                    self.flow_cache[init_key+','+goal_key] = final_parts

            i += 1
       
        return self.flow_cache

    def shrinkPolytope(self, poly):
        try:
            print('Shrinking polytope')
            A_matrix = poly.A
            b_vector = poly.b

            new_A_matrix = A_matrix
            new_b_matrix = b_vector

            for row in range(len(A_matrix)):
                A_row = A_matrix[row]
                A_row_norm = np.linalg.norm(A_row)

                b_row = b_vector[row]
                new_b_matrix[row] = b_row - self.shrinking_constant*A_row_norm

            new_poly = pc.Polytope(new_A_matrix, new_b_matrix)
            return new_poly

        except:
            print("Can't shrink polytope!")
            return None

    def exampleRun(self, num_cycles = 3):
        if len(self.buchi_inits) > 1:
            print("Too many possible initial states!") #TODO: NEED TO UPDATE THIS PART
            return None
        else:
            init_states = self.transitions_in_dict[self.buchi_inits[0]]

            if self.buchi_run['prefix'] != []:
                init_transition = self.buchi_run['prefix'][0][0]
            else:
                init_transition = self.buchi_run['cycle'][0][0]

            init_label = random.choice(init_states)
            while init_label[0] == init_transition:
                init_label = random.choice(init_states)
            
            sample_run = [init_label[0]]
            for transition in self.buchi_run['prefix']:
                sample_run.append(transition[0])

            curr_cycle = 1
            while curr_cycle <= num_cycles:
                curr_cycle += 1
                for transition in self.buchi_run['cycle']:
                    sample_run.append(transition[0])

            return sample_run

