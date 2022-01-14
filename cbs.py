import copy
import time as timer
import heapq
import random
from collections import defaultdict

from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, get_shortest_paths, \
    get_shortest_paths_bfs, check_path_constraints


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    # the longer path is chosen to compare with
    # then each location on both paths is compared, and if there are any collisions
    # they get stored in a dictionary and returned.
    # if there is no collisions then None is returned

    if len(path1) > len(path2):
        length = len(path1)
    else:
        length = len(path2)

    for i in range(length):
        if get_location(path1, i) == get_location(path2, i):
            return {'a1': -1, 'a2': -1, 'loc': [get_location(path1, i)], 'timestep': i}
        if get_location(path1, i - 1) == get_location(path2, i) and get_location(path1, i) == get_location(path2,
                                                                                                           i - 1):
            return {'a1': -1, 'a2': -1, 'loc': [get_location(path1, i - 1), get_location(path1, i)], 'timestep': i}
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collision_list = []

    # Every combination of agents is compared for collisions
    # detect_collision is called to check each pair of agents
    # collisions are appended to a list and returned.
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):

            collision = detect_collision(paths[i], paths[j])

            if collision is not None:
                collision['a1'] = i
                collision['a2'] = j
                collision_list.append(collision)
                continue

    return collision_list


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    # First, collision is checked to see if it is a vertex or edge constraint.
    # Then, a pair of constraints is created accordingly and returned.
    constraint_type = len(collision['loc'])

    if constraint_type == 1:
        first_constraint = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']}
        second_constraint = {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep']}
        constraint_list = [first_constraint, second_constraint]
        return constraint_list

    if constraint_type == 2:
        first_constraint = {'agent': collision['a1'], 'loc': [collision['loc'][0], collision['loc'][1]],
                            'timestep': collision['timestep']}
        second_constraint = {'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]],
                             'timestep': collision['timestep']}
        constraint_list = [first_constraint, second_constraint]
        return constraint_list


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    # this function works almost the same as standard splitting, but 1 agent is chosen and randomly
    # and both constraints are made for the same agent, one positive and one negative

    constraint_list = []
    constraint_type = len(collision['loc'])
    random_number = random.randint(0, 1)

    if random_number == 0:
        if constraint_type == 1:
            first_constraint = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'],
                                'positive': True}
            second_constraint = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'],
                                 'positive': False}
            constraint_list = [first_constraint, second_constraint]

        if constraint_type == 2:
            first_constraint = {'agent': collision['a1'], 'loc': [collision['loc'][0], collision['loc'][1]],
                                'timestep': collision['timestep'], 'positive': True}
            second_constraint = {'agent': collision['a1'], 'loc': [collision['loc'][0], collision['loc'][1]],
                                 'timestep': collision['timestep'], 'positive': False}
            constraint_list = [first_constraint, second_constraint]

    if random_number == 1:
        if constraint_type == 1:
            first_constraint = {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'],
                                'positive': True}
            second_constraint = {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'],
                                 'positive': False}
            constraint_list = [first_constraint, second_constraint]

        if constraint_type == 2:
            first_constraint = {'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]],
                                'timestep': collision['timestep'], 'positive': True}
            second_constraint = {'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]],
                                 'timestep': collision['timestep'], 'positive': False}
            constraint_list = [first_constraint, second_constraint]

    return constraint_list


def build_mdd(path_list):
    mdd = defaultdict(list)
    # print(path_list)
    mdd[0] = [path_list[0][0]]
    number_of_timesteps = len(path_list[0])
    number_of_paths = len(path_list)

    for i in range(1, number_of_timesteps):
        for j in range(number_of_paths):
            path = []
            for k in range(0, i + 1):
                path.append(path_list[j][k])
            if path in mdd[i]:
                continue
            mdd[i].append(path)

    return mdd


def compare_mdd(mdd_list):
    conflicts = dict()

    for i in range(len(mdd_list) - 1):
        if mdd_list[i] == 0:
            continue
        for j in range(i + 1, len(mdd_list)):
            if mdd_list[j] == 0:
                continue
            key_list = mdd_list[i]
            for k in range(1, len(key_list)):
                if k in mdd_list[j]:
                    level_k_i = mdd_list[i].get(k)
                    level_k_j = mdd_list[j].get(k)
                    last_element = level_k_i[0][-1]
                    for l in range(len(level_k_i)):
                        if l in range(len(level_k_j)):
                            conflict = cardinal_conflicts(level_k_i, level_k_j, i, j, k)
                            if conflict is not None:
                                key = list(conflict)[0]
                                if key not in conflicts:
                                    conflicts[key] = conflict.get(key)
                                else:
                                    item = conflict.get(key)
                                    dict_list = conflicts.get(key)
                                    dict_item = {}
                                    for dict_item in dict_list:
                                        if item[0]['type'] in dict_item:
                                            agent_list = dict_item['agent']
                                            if i not in agent_list:
                                                agent_list.append(i)
                                            if j not in agent_list:
                                                agent_list.append(j)
                                            continue
                                    dict_list.append(dict_item)
    conflicts2 = dict()
    keys = conflicts.keys()
    for i in keys:
        first = conflicts[i][0]
        conflicts2[i] = [first]
    return conflicts2


def cardinal_conflicts(level1, level2, agent1, agent2, timestep):
    if len(level1) >= len(level2):
        max_level = level1
        min_level = level2
    else:
        max_level = level2
        min_level = level1

    flag1 = 0
    flag2 = 0
    flag3 = 0

    conflict_type = 'vertex'
    # check for non-cardinal conflicts
    non_cardinal = {}
    for i in range(len(max_level)):
        for j in range(len(min_level)):
            if max_level[i][-1] == min_level[j][-2] and min_level[j][-1] == max_level[i][-2]:
                conflict_type = 'edge'
            if max_level[i][-1] == min_level[j][-1]:
                flag3 = flag3 + 1
                non_cardinal = {(timestep, max_level[i][-1]): [
                    {'agent': [agent1, agent2], 'type': 'non-cardinal', 'conflict': conflict_type}]}

    if flag3 == 1:
        return non_cardinal

    # check for semi & cardinal conflicts
    cc = list()
    sc = list()

    last_element_max = max_level[0][-1]

    for i in range(len(max_level)):
        # print("i: ", i)
        if max_level[i][-1] == last_element_max:
            # print("max_level[i][-1]", max_level[i][-1])
            # print("last_element_max", last_element_max)
            flag1 = flag1 + 1
            # print("flag1: ", flag1)
            if i == len(max_level) - 1 and flag1 == len(max_level):
                cc = max_level
                sc = min_level
        else:
            flag1 = 0
        if i < len(min_level):
            if min_level[i][-1] == last_element_max:
                flag2 = flag2 + 1
                if i == len(min_level) - 1 and flag2 == len(min_level):
                    cc = min_level
                    sc = max_level
                    continue
            else:
                flag2 = 0

    if flag1 == len(max_level) and flag2 == len(min_level):
        cardinal = {
            (timestep, last_element_max): [{'agent': [agent1, agent2], 'type': 'cardinal', 'conflict': conflict_type}]}
        # print("cardinal: ")
        return cardinal

    if (flag1 == len(max_level) and flag2 == 1) or (flag2 == len(min_level) and flag1 == 1):
        for j in range(len(sc)):
            if cc[0][-1] == sc[j][-1]:
                semi_cardinal = {
                    (timestep, cc[0][-1]): [{'agent': [agent1, agent2], 'type': 'semi', 'conflict': conflict_type}]}
                # print("semi_cardinal")
                return semi_cardinal
    return None


def build_CG(conflicts):
    # print("build_CG", conflicts)

    # input: dictionary indexed by (timestep, location) = list of agents
    conflict_graph = list()
    for conflict_list in conflicts.values():
        # print("conflict_list", conflict_list)
        for conflict in conflict_list:
            # print("cgf", conflict)
            if conflict['type'] == 'cardinal':
                conflict_graph.append(conflict['agent'])
    print("graph", conflict_graph)
    return conflict_graph


# def build_CG(conflicts):
#     # print("build_CG", conflicts)
#
#     # input: dictionary indexed by (timestep, location) = list of agents
#     conflict_graph = list()
#     keys = len(conflicts.keys())
#     count = 0
#
#     for conflict_list in conflicts.values():
#         # print("conflict_list", conflict_list)
#         count = 0
#         for conflict in conflict_list:
#             if count >= keys:
#                 continue
#             # print("cgf", conflict)
#             if conflict['type'] == 'cardinal':
#                 conflict_graph.append(conflict['agent'])
#                 count = count + 1
#     # print("graph", conflict_graph)
#
#     # for i in range(len(conflicts.keys())):
#     #
#     #     if conflicts[i]['type'] == 'cardinal':
#     #         conflict_graph.append(conflicts[i]['agent'])
#     # # print("graph", conflict_graph)
#     # return conflict_graph
#     return conflict_graph


def compute_mvc(conflicts):
    if len(conflicts) == 1:
        return 1
    mvc = 0
    for cg in conflicts:
        mvc = mvc + len(cg) - 1

    return mvc


def build_DG(conflicts):
    dependency_graph = list()
    for conflict_list in conflicts.values():
        for conflict in conflict_list:
            if conflict['type'] == 'semi-cardinal' or conflict['type'] == 'non-cardinal':
                dependency_graph.append(conflict['agent'])
    return dependency_graph


def build_WDG(dependency_graph, conflict_graph, shortest_paths, paths):
    weighted_dg = dependency_graph + conflict_graph
    wdg = dict()

    for agent_list in weighted_dg:
        for i in range(len(agent_list) - 1):
            sum_paths = 0
            for j in range(i + 1, len(agent_list)):
                sum_paths = len(paths[agent_list[i]]) - 1 + len(paths[agent_list[j]]) - 1
                path_list_i = shortest_paths.get(agent_list[i])
                path_list_j = shortest_paths.get(agent_list[j])
                sum_shortest_paths = 0
                if path_list_i is not None and path_list_j is not None:
                    sum_shortest_paths = len(path_list_i[0]) - 1 + len(path_list_j[0]) - 1
                for_agents = list()
                for_agents.append(copy.deepcopy(agent_list[i]))
                for_agents.append(copy.deepcopy(agent_list[j]))
                difference = sum_paths - sum_shortest_paths
                # if difference == 0:
                #     difference = 1
                # print("path_diff", difference)
                if difference >= 0:
                    entry = {'agents': for_agents, 'weight': difference}
                    if tuple(agent_list) in wdg:
                        graph_list = copy.deepcopy(wdg.get(tuple(agent_list)))
                        graph_list.append(copy.deepcopy(entry))
                    else:
                        wdg[tuple(agent_list)] = copy.deepcopy([entry])

    return wdg


def wdg_heuristic(wdg):
    if len(wdg) == 0:
        return 0
    vertex_values = list()
    heuristic = 0

    third_vertex_a = 0
    third_vertex_b = 0
    values = list()
    difference = 0

    for key in wdg.keys():
        graph_list = copy.deepcopy(wdg.get(key))
        for i in range(len(graph_list)):
            agents = copy.deepcopy(graph_list[i]['agents'])
            if key[0] in agents and key[1] in agents:
                weight = copy.deepcopy(graph_list[i]['weight'])
                difference = weight - 1
                values.append(difference)
                values.append(1)
            if len(key) == 2:
                vertex_values.append(values)
                values = list()
                continue

        if len(key) == 3:
            for j in range(len(graph_list)):
                agents = copy.deepcopy(graph_list[j]['agents'])
                if key[0] in agents and key[2] in agents:
                    weight1 = copy.deepcopy(graph_list[j]['weight'])
                    third_vertex_a = weight1 - difference
                if key[1] in agents and key[2] in agents:
                    weight1 = copy.deepcopy(graph_list[j]['weight'])
                    third_vertex_b = weight1 - difference
            if third_vertex_a > 0:
                values.append(third_vertex_a)
            elif third_vertex_b > 0:
                values.append(third_vertex_b)
            vertex_values.append(values)
            values = list()
            
    for i in range(len(vertex_values)):
        heuristic = heuristic + sum(vertex_values[i])

    return heuristic


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node, h_value):
        if h_value == 0:
            heapq.heappush(self.open_list,
                           (h_value + node['cost'], node['cost'], len(node['collisions']), self.num_of_generated, node))
        else:
            heapq.heappush(self.open_list,
                           (h_value + node['cost'], node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True, cg=True, dg=True, wdg=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        current_path_length = list()
        previous_agent_paths = list()
        current_mdd_length = dict()
        for i in range(self.num_of_agents):
            current_mdd_length[i] = 0
            current_path_length.append(-1)
            previous_agent_paths.append(-1)

        # this is only here for testing atm
        # path_list = get_shortest_paths(self.my_map, self.starts[1], self.goals[1], self.heuristics[1],
        #                                1, [])
        # print("path list:")
        # print(path_list)

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        # print("self.heuristics: ", self.heuristics)
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root, 0)

        # Task 3.1: Testing
        print("root Collisions: ")
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print("root constraints:")
            if disjoint:
                print(disjoint_splitting(collision))
            else:

                print(standard_splitting(collision))
        print("root: ", root)
        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        # Each time a node is removed, if there are no collisions it is returned as the goal path.
        # If not, the node's first collision is converted into constraints
        # For each of these constraints a new node is made, with new constraints added to the old ones
        # a_star is used to find a path with the new constraints, and if a path exists it is added to the open_list.
        # Eventually the node with the lowest cost and no collisions will be popped and chosen as the goal

        # Part 4: added if statements to check if code was run with --disjoint flag, runs accordingly
        # Main difference is after a path is found, other agents need to be checked
        # after path is found with positive constraint, negative constraints are added for all other agents acordingly

        while len(self.open_list) > 0:

            #redacted

                end_time = timer.time()

                print("Time build mdd:", end_time - start_time)
                # print(mdd_list)
                # compare mdd's and so on
                # ......

                #
                # heuristic_cg = compute_mvc(conflict_graph)
                # print("heuristic_cg", heuristic_cg)

                start_time1 = timer.time()
                cardinal_conflicts = compare_mdd(mdd_list)
                print("cardinal_conflicts", cardinal_conflicts)
                end_time1 = timer.time()
                print("Time cardinal conflicts:", end_time1 - start_time1)

                start_time2 = timer.time()
                conflict_graph = build_CG(cardinal_conflicts)
                print("conflict_graph", conflict_graph)
                end_time2 = timer.time()
                print("Time conflict graph:", end_time2 - start_time2)

                start_time3 = timer.time()
                heuristic_cg = compute_mvc(conflict_graph)
                print("heuristic_cg", heuristic_cg)
                end_time3 = timer.time()
                print("Time heuristic_cg:", end_time3 - start_time3)

            dependency_graph = list()
            if dg or wdg:
                dependency_graph = build_DG(cardinal_conflicts)
                heuristic_dg = compute_mvc(dependency_graph) + heuristic_cg
                print("heuristic_dg", heuristic_dg)

            if wdg:

                wdg = build_WDG(dependency_graph, conflict_graph, agent_paths, current_node['paths'])
                heuristic_wdg = wdg_heuristic(wdg)
                print("heuristic_wdg", heuristic_wdg)

            collision = current_node['collisions'][0]

            if disjoint:
                constraints = disjoint_splitting(collision)
            else:
                constraints = standard_splitting(collision)

            for i in range(len(constraints)):

                child_constraints = copy.deepcopy(current_node['constraints'])
                child_path = copy.deepcopy(current_node['paths'])

                # check what type of conflict (if any), get h value

                child = {'cost': 0,
                         'constraints': child_constraints,
                         'paths': child_path,
                         'collisions': []}

                check = 0
                for k in range(len(child['constraints'])):
                    if constraints[i] == child['constraints'][k]:
                        check = 1

                if check == 1:
                    continue
                else:
                    child['constraints'].append(constraints[i])

                agent = constraints[i]['agent']

                new_path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                                  agent, child['constraints'])

                # print("new_path", new_path)

                if new_path is not None:
                    bad_node = False
                    if disjoint:

                        if constraints[i]['positive']:

                            for k in range(len(child['paths'])):
                                positive_constraint = copy.deepcopy(constraints[i])

                                if k == constraints[i]['agent']:
                                    continue

                                positive_constraint['agent'] = copy.deepcopy(k)
                                positive_constraint['positive'] = False

                                if len(positive_constraint['loc']) == 1:
                                    child['constraints'].append(positive_constraint)

                                if len(positive_constraint['loc']) == 2:
                                    second_positive_constraint = copy.deepcopy(positive_constraint)
                                    temp = second_positive_constraint['loc'][0]
                                    temp2 = second_positive_constraint['loc'][1]
                                    second_positive_constraint['loc'][0] = temp2
                                    second_positive_constraint['loc'][1] = temp
                                    child['constraints'].append(second_positive_constraint)

                            violation_list = paths_violate_constraint(constraints[i], child['paths'])
                            for j in range(len(violation_list)):

                                new_agent_path = a_star(self.my_map, self.starts[violation_list[j]],
                                                        self.goals[violation_list[j]],
                                                        self.heuristics[violation_list[j]],
                                                        violation_list[j], child['constraints'])

                                if new_agent_path is None:
                                    bad_node = True

                                else:
                                    child['paths'][violation_list[j]] = copy.deepcopy(new_agent_path)
                    if not bad_node:
                        child['paths'][agent] = new_path
                        child['collisions'] = detect_collisions(child['paths'])
                        child['cost'] = get_sum_of_cost(child['paths'])
                        print(child)
                        if cg:
                            print("bbb")
                            self.push_node(child, heuristic_cg)
                        if dg:
                            print("dfdfdf")
                            self.push_node(child, heuristic_dg)
                        if wdg:
                            print("fgfgfg")
                            self.push_node(child, heuristic_wdg)
                        if not cg and not dg and not wdg:
                            print("ghghgh")
                            self.push_node(child, 0)

        return 'No Solutions'
        # print()
        # self.print_results(root)
        # return root['paths']

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
