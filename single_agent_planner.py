import copy
import heapq
from collections import defaultdict


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']

    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    # deafultdict is used so we can add multiple constraints for the given timestep
    # first we make a list with just the constraints for that specific agent
    duplicate = []
    constraint_table = defaultdict(list)

    for i in range(len(constraints)):
        if 'positive' not in constraints[i]:
            constraints[i]['positive'] = False

    for i in range(len(constraints)):
        if constraints[i]['agent'] == agent:
            duplicate.append(constraints[i])

    # then, for all of the constraints for that agent, they are added to the defaultdict
    # these are checked to make sure they are done in order
    while len(duplicate) > 0:

        current = duplicate[0]['timestep']
        current_index = 0
        for i in range(len(duplicate)):

            if duplicate[i]['timestep'] < current:
                current = duplicate[i]['timestep']
                current_index = i

        constraint_table[current].append(duplicate[current_index])
        duplicate.pop(current_index)

    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    # checks the constraint table for the given time step.
    # then checks if constraint is vertex constraint or edge constraint
    # then checks if constraint is violated accordingly, return True if yes, False if no violation
    # in part 4 I added a second block to check for positive constraints
    # this code makes sure the positive constraint is being met

    for i in range(len(constraint_table[next_time])):
        if constraint_table[next_time][i]['positive']:
            if len(constraint_table[next_time][i]['loc']) == 2:
                if curr_loc != constraint_table[next_time][i]['loc'][0] or next_loc != \
                        constraint_table[next_time][i]['loc'][1]:
                    return True
            else:
                if next_loc != constraint_table[next_time][i]['loc'][0]:
                    return True

        if not constraint_table[next_time][i]['positive']:
            if len(constraint_table[next_time][i]['loc']) == 2:
                if curr_loc == constraint_table[next_time][i]['loc'][0] and next_loc == \
                        constraint_table[next_time][i]['loc'][1]:
                    return True
            else:
                if next_loc == constraint_table[next_time][i]['loc'][0]:
                    return True
    return False


def push_node_gsp(open_list, node):
    # print()
    # print(node)
    # print(open_list)

    # heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['g_val'], node['loc'], node))
    index = 1
    node_holder = {0: index, index: node}
    check = 0
    loop_check = 1
    # while check == -1:
    #     try:
    #         print("before err")
    #         print(node)
    #         print(node['g_val'])
    #         print(node['h_val'])
    open_list_length = len(open_list)
    while loop_check == 1:
        current_node_values = [node['g_val'] + node['h_val'], node['h_val'], node['loc'], node_holder[0]]

        if open_list_length == 0:
            check = 0
        else:
            for i in range(len(open_list)):
                heap_test = [open_list[i][0], open_list[i][1], open_list[i][2], open_list[i][3]]

                if heap_test == current_node_values:
                    check = -1

        if check == 0:
            heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'],
                                       node_holder[0], node_holder))
            loop_check = 0
            continue

        if check == -1:
            index = index + 1
            node_holder = {0: index, index: node}
            check = 0

    # print("after")
    #
    # print(open_list)
    #     # except TypeError:
    #
    #         print("ERROR")
    #         print(node)
    #         print(open_list)
    #         check = -1
    #         index = index + 1
    #         node_holder = {0: index, index: node}


def pop_node_gsp(open_list):
    # print("open list")
    # print(open_list)
    current = heapq.heappop(open_list)
    curr = current[4]
    index = curr[0]
    # print(index)
    current_node = curr[index]
    # # print(current_node)
    # _, _, _, _, curr = heapq.heappop(open_list)
    return current_node


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def get_distance(child_loc, goal_loc):
    x_distance = child_loc[0] - goal_loc[0]
    y_distance = child_loc[1] - goal_loc[1]
    if x_distance < 0:
        x_distance = x_distance * -1
    if y_distance < 0:
        y_distance = y_distance * -1
    distance = x_distance + y_distance
    return distance


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    map_length = len(my_map) - 1
    map_width = len(my_map[0]) - 1

    total_spaces = 0
    for i in range(len(my_map)):
        for j in range(len(my_map[i])):
            if not my_map[i][j]:
                total_spaces = total_spaces + 1
    max_time = total_spaces * total_spaces

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    constraint_table = build_constraint_table(constraints, agent)
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)

        if curr['timestep'] > max_time:
            return None

        # This is where the negative timestep constraints are processed.
        # If a constraint with a negative timestep is found, that timestep is added to a list.
        # Then, a constraint is added for the current agent at the next timestep
        # These negative timestep constraints make it so that a previous agent's goal location
        # will always be added as a constraint.

        count = []
        for i in constraint_table.keys():
            if i < 0 and curr['timestep'] >= (i * -1):
                count.append(i)
        for i in range(len(count)):
            constraint_table[curr['timestep'] + 1].append(
                {'agent': agent, 'loc': constraint_table[count[i]][0]['loc'], 'timestep': curr['timestep'] + 1,
                 'positive': False})
        #############################

        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc:

            # every constraint is checked
            # if there is one matching the goal location, its checked to see if it is in the future
            # if yes, path is not goal path. if no, path is returned as goal node.
            # In part 4 I added some code to check positive constraints
            # because future constraints will behave differently when they're positive

            test_variable = 0
            for i in constraint_table.keys():
                if constraint_table[i]:
                    for j in range(len(constraint_table[i])):

                        if constraint_table[i][j]['timestep'] >= curr['timestep']:

                            if not constraint_table[i][j]['positive']:
                                if len(constraint_table[i][j]['loc']) == 1:
                                    if constraint_table[i][j]['loc'][0] == goal_loc:
                                        test_variable = 1

                            if constraint_table[i][j]['positive']:
                                if len(constraint_table[i][j]['loc']) == 2:
                                    if constraint_table[i][j]['loc'][1] != goal_loc:
                                        test_variable = 1

            if test_variable == 0:
                return get_path(curr)

        for dir in range(5):
            child_loc = move(curr['loc'], dir)

            if child_loc[0] < 0 or child_loc[1] < 0:
                continue
            if child_loc[0] > map_length or child_loc[1] > map_width:
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'timestep': curr['timestep'] + 1}
            if is_constrained(curr['loc'], child['loc'], child['timestep'], constraint_table):
                continue

            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions


def get_shortest_paths_bfs(my_map, start_loc, goal_loc, h_values, agent, constraints, cost):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    path_list = []
    map_length = len(my_map) - 1
    map_width = len(my_map[0]) - 1

    open_list = []
    closed_list = defaultdict(list)
    closed_list2 = []

    h_value = h_values[start_loc]
    constraint_table = build_constraint_table(constraints, agent)
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}

    closed_list[(root['loc'], root['timestep'])] = root
    open_list.append(root)
    closed_list2.append(get_path(root))
    while len(open_list) > 0:

        curr = open_list.pop(0)

        count = []
        for i in constraint_table.keys():
            if i < 0 and curr['timestep'] >= (i * -1):
                count.append(i)
        for i in range(len(count)):
            constraint_table[curr['timestep'] + 1].append(
                {'agent': agent, 'loc': constraint_table[count[i]][0]['loc'], 'timestep': curr['timestep'] + 1,
                 'positive': False})

        if curr['loc'] == goal_loc:

            test_variable = 0
            for i in constraint_table.keys():
                if constraint_table[i]:
                    for j in range(len(constraint_table[i])):

                        if constraint_table[i][j]['timestep'] >= curr['timestep']:

                            if not constraint_table[i][j]['positive']:
                                if len(constraint_table[i][j]['loc']) == 1:
                                    if constraint_table[i][j]['loc'][0] == goal_loc:
                                        test_variable = 1

                            if constraint_table[i][j]['positive']:
                                if len(constraint_table[i][j]['loc']) == 2:
                                    if constraint_table[i][j]['loc'][1] != goal_loc:
                                        test_variable = 1

            if test_variable == 0:

                if cost == curr['timestep']:
                    path = get_path(curr).copy()
                    path_list.append(path)

        for dir in range(5):
            child_loc = move(curr['loc'], dir)

            if child_loc[0] < 0 or child_loc[1] < 0:
                continue
            if child_loc[0] > map_length or child_loc[1] > map_width:
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue

            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'timestep': curr['timestep'] + 1}

            # print(closed_list2)
            # print(get_path(child))
            if get_path(child) in closed_list2:
                print("fff")
                continue

            current_distance = get_distance(child['loc'], goal_loc)
            current_timestep = child['timestep']
            remaining_time = cost - current_timestep

            if remaining_time < current_distance:
                continue

            if child['timestep'] > cost:
                continue

            if (child['g_val'] + child['h_val']) > cost:
                continue

            if child['timestep'] - 1 > cost:
                print(cost)
                print(child['timestep'])
                return path_list

            # if (child['g_val'] + child['h_val']) == cost and child['loc'] != goal_loc:
            #     continue

            # if child['timestep'] == cost:
            #     if child['loc'] != goal_loc:
            #         continue

            if is_constrained(curr['loc'], child['loc'], child['timestep'], constraint_table):
                continue

            # if (child['loc'], child['timestep']) in closed_list:
            #
            #     continue


            closed_list2.append(get_path(child))
            # closed_list[(child['loc'], child['timestep'])] = child
            open_list.append(child)

    return path_list


def check_path_constraints(path_list, constraints, agent, cost):
    new_path_list = list()
    constraint_table = build_constraint_table(constraints, agent)

    for i in range(len(path_list)):
        test_variable = 0

        length = len(path_list[i])

        for j in range(len(path_list[i])):

            if test_variable == 1:
                continue
            if j + 1 >= length:
                continue

            if is_constrained(path_list[i][j], path_list[i][j + 1], j + 1, constraint_table):

                test_variable = 1

            for k in constraint_table.keys():
                if constraint_table[k]:
                    for l in range(len(constraint_table[k])):

                        if constraint_table[k][l]['timestep'] >= cost:

                            if not constraint_table[k][l]['positive']:
                                if len(constraint_table[k][l]['loc']) == 1:
                                    if constraint_table[k][l]['loc'][0] == path_list[i][cost]:
                                        test_variable = 1

                            if constraint_table[k][l]['positive']:
                                if len(constraint_table[k][l]['loc']) == 2:
                                    if constraint_table[k][l]['loc'][1] != path_list[i][cost]:
                                        print(constraint_table[k][l]['timestep'])
                                        print(constraint_table[k][l]['loc'][1])
                                        print(path_list[i][cost])
                                        print("bad")
                                        test_variable = 1
            if test_variable == 1:
                continue

        if test_variable == 0:
            new_path_list.append(path_list[i])
    return new_path_list


def get_shortest_paths(my_map, start_loc, goal_loc, h_values, agent, constraints, cost):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    path_list = []
    # current_cost = cost
    map_length = len(my_map) - 1
    map_width = len(my_map[0]) - 1

    total_spaces = 0
    for i in range(len(my_map)):
        for j in range(len(my_map[i])):
            if not my_map[i][j]:
                total_spaces = total_spaces + 1
    max_time = total_spaces * total_spaces

    open_list = []
    closed_list = defaultdict(list)
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    constraint_table = build_constraint_table(constraints, agent)
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node_gsp(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:

        curr = pop_node_gsp(open_list)
        if curr['timestep'] > max_time:
            return None

        # This is where the negative timestep constraints are processed.
        # If a constraint with a negative timestep is found, that timestep is added to a list.
        # Then, a constraint is added for the current agent at the next timestep
        # These negative timestep constraints make it so that a previous agent's goal location
        # will always be added as a constraint.

        count = []
        for i in constraint_table.keys():
            if i < 0 and curr['timestep'] >= (i * -1):
                count.append(i)
        for i in range(len(count)):
            constraint_table[curr['timestep'] + 1].append(
                {'agent': agent, 'loc': constraint_table[count[i]][0]['loc'], 'timestep': curr['timestep'] + 1,
                 'positive': False})
        #############################

        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc:
            # print(get_path(curr))
            # every constraint is checked
            # if there is one matching the goal location, its checked to see if it is in the future
            # if yes, path is not goal path. if no, path is returned as goal node.
            # In part 4 I added some code to check positive constraints
            # because future constraints will behave differently when they're positive

            test_variable = 0
            for i in constraint_table.keys():
                if constraint_table[i]:
                    for j in range(len(constraint_table[i])):

                        if constraint_table[i][j]['timestep'] >= curr['timestep']:

                            if not constraint_table[i][j]['positive']:
                                if len(constraint_table[i][j]['loc']) == 1:
                                    if constraint_table[i][j]['loc'][0] == goal_loc:
                                        test_variable = 1

                            if constraint_table[i][j]['positive']:
                                if len(constraint_table[i][j]['loc']) == 2:
                                    if constraint_table[i][j]['loc'][1] != goal_loc:
                                        test_variable = 1

            if test_variable == 0:

                # if current_cost == -1:
                #     current_cost = curr['timestep']

                if cost == curr['timestep']:
                    path = get_path(curr).copy()
                    path_list.append(path)

        for dir in range(5):
            child_loc = move(curr['loc'], dir)

            if child_loc[0] < 0 or child_loc[1] < 0:
                continue
            if child_loc[0] > map_length or child_loc[1] > map_width:
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'timestep': curr['timestep'] + 1}

            # print("time and cost")
            # print(child['timestep'])
            # print(current_cost)

            current_distance = get_distance(child['loc'], goal_loc)
            current_timestep = child['timestep']

            remaining_time = cost - current_timestep
            # print(current_timestep)
            # print(current_distance)
            # print(cost)

            if remaining_time < current_distance:
                continue

            if child['timestep'] > cost:
                continue

            if (child['g_val'] + child['h_val']) > cost:
                continue

            # if (child['g_val'] + child['h_val']) == cost and child['loc'] != goal_loc:
            #     continue

            # if child['timestep'] == cost:
            #     if child['loc'] != goal_loc:
            #         continue

            if is_constrained(curr['loc'], child['loc'], child['timestep'], constraint_table):
                continue

            push_node_gsp(open_list, child)
            # print("here")
            # print(open_list)
            # print(get_path(child))
            # print(path_list)
            # if get_path(child) in path_list:
            #     continue

            # test_variable2 = 0
            # for i in range(len(closed_list[(child['loc'], child['timestep'])])):
            #
            #     done_path = get_path(closed_list[(child['loc'], child['timestep'])][i])
            #     current_path = get_path(child)
            #     if done_path == current_path:
            #         test_variable2 = -1

            # if test_variable2 == 0:
            #     closed_list[(child['loc'], child['timestep'])].append(child)
            #     push_node(open_list, child)
            # else:
            #     continue
            # if (child['loc'], child['timestep']) in closed_list:
            #     # print("closed list:")
            #     # print(closed_list)
            #     # print(get_path(child))
            #     # existing_node = get_path(closed_list[(child['loc'], child['timestep'])])
            #     # print("existing_node")
            #     # print(existing_node)
            # if child['timestep'] < 5:
            # print("find me")
            # print(done_path)
            # print(current_path)
            # print(closed_list)
            #     print(get_path(closed_list[(child['loc'], child['timestep'])]))
            #     print(get_path(child))
            #     if get_path(closed_list[(child['loc'], child['timestep'])]) == (get_path(child)):
            #         print("good")
            #         continue

            # if compare_nodes(child, existing_node):
            #     closed_list[(child['loc'], child['timestep'])] = child

            # else:
            #
            #     test = -1
            #     # print("existing node:")
            #     # print(existing_node['g_val'] + existing_node['h_val'], existing_node['h_val'], existing_node['loc'], existing_node)
            #     # print(child['g_val'] + child['h_val'], child['h_val'], child['loc'], child)
            #
            #     while test == -1:
            #         print("before")
            #         print(child)
            #         print(existing_node)
            #         existing_node['h_val'] = existing_node['h_val'] + 1
            #         print("after")
            #         print(child)
            #         print(existing_node)
            #         if compare_nodes(child, existing_node):
            #             closed_list[(child['loc'], child['timestep'])] = child
            #             push_node(open_list, child)
            #             test = 0

            # else:
            #     closed_list[(child['loc'], child['timestep'])].append(child)
            #     push_node(open_list, child)

    return path_list
