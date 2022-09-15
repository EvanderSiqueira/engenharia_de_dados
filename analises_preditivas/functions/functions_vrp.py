
import gmaps
import time
import random

# chave google maps
def google_maps_api_key():
    return ""

# criando_modelo =======================================================================================================

def criando_modelo(distance_matrix, demanda, capacidade_veiculos):
  
    data = {}
    data['distance_matrix'] = distance_matrix
    data['num_vehicles'] = 10
    data['depot'] = 0
    data['demands'] = demanda  
    data['vehicle_capacities'] =  capacidade_veiculos
    
    return data

# imprimindo_solucao =======================================================================================================

def imprimindo_solucao(data, manager, routing, solution):
        
    # Display dropped nodes.
    dropped_nodes = 'Pontos de entregas não realizados:'
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(node)) == node:
            dropped_nodes += ' {}'.format(manager.IndexToNode(node))
    print(dropped_nodes)
    # Display routes
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Rota do veículo {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):

            node_index = manager.IndexToNode(index)

            route_load += data['demands'][node_index]
            peso_entrega = data['demands'][node_index]
            
            if node_index == 0:
                plan_output += 'Ponto {0}'.format(node_index)
            else:
                plan_output += ' -> Ponto {0} (descarregar: {1} quilos)'.format(node_index, peso_entrega)

            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
    
        plan_output += '\nDistância da rota: {}m\n'.format(route_distance)
        plan_output += 'Carga Total: {}\n'.format(route_load)
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
    print('Distância total de todas as rotas: {}m'.format(total_distance))
    print('Carga total de todas as rotas: {}'.format(total_load))

# obtendo rotas =======================================================================================================

def obtendo_rotas(num_vehicles, manager, routing, solution):
    routes = {}
    for vehicle_id in range(num_vehicles):
        routes[vehicle_id] = []
        index = routing.Start(vehicle_id)
        while not routing.IsEnd(index):
            routes[vehicle_id].append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
        routes[vehicle_id].append(manager.IndexToNode(index))
    return routes

# solver_rotas =======================================================================================================

def solver_rotas(data, manager, routing, penalty, pywrapcp, routing_enums_pb2):
    
    """Solve the CVRP problem."""

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')
    
    # Allow to drop nodes.
    # penalty = 10
    for node in range(1, len(data['distance_matrix'])):
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(1)
    
    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    return solution




# gerar mapa de pontos =======================================================================================================


def gerar_mapa_pontos(titulo, localizacao_base, nome_base, localizacao_entregas, nome_entregas):
  # criando o ponto do armazem
  layer_armazem = gmaps.symbol_layer([localizacao_base], hover_text=nome_base, fill_color='white', info_box_content=nome_base, scale=8, stroke_color='blue')

  # criando os pontos das entregas
  layer_entregas = gmaps.symbol_layer(localizacao_entregas, hover_text=nome_entregas, info_box_content=nome_entregas, fill_color='white', stroke_color='black', scale=4)

  fig = gmaps.figure()
  fig.add_layer(layer_armazem)
  fig.add_layer(layer_entregas)
  print(titulo)
  
  return fig

# gerar direction layer =======================================================================================================
def criando_direction_layer(localizacao_base, localizacao_rotas, markers):
    
    # gerando cores aleatórias para a rota de cada veiculo
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    cor = '#%02x%02x%02x' % (r, g, b)
    
    # ultimo ponto
    u = localizacao_rotas[-1]

    # outros pontos que não a base e o ultimo
    w = localizacao_rotas[1:-1]
    time.sleep(3)
    if w:
      dl = gmaps.directions_layer(localizacao_base, u, waypoints=w, stroke_color=cor, show_markers=markers, stroke_opacity=0.8)
    else:
      dl = gmaps.directions_layer(localizacao_base, u, stroke_color=cor, show_markers=markers, stroke_opacity=0.8)

    return dl

# mapa das rotas =======================================================================================================
def criando_mapa_com_rotas(titulo, mapa, localizacao_base, localizacao_entregas, rotas_mapa):
    localizacao_rotas = []
    
    if isinstance(rotas_mapa, list):
      for id_entrega in rotas_mapa:
        localizacao_rotas.append(localizacao_entregas[id_entrega])
      
      localizacao_rotas = localizacao_rotas[:-1]
      dl = criando_direction_layer(localizacao_base, localizacao_rotas, True)
      mapa.add_layer(dl)

    else:
      for id_veiculo in rotas_mapa:
        for id_entrega in rotas_mapa[id_veiculo]:
          localizacao_rotas.append(localizacao_entregas[id_entrega])

        localizacao_rotas = localizacao_rotas[:-1]
        dl = criando_direction_layer(localizacao_base, localizacao_rotas, False)
        time.sleep(3)
        mapa.add_layer(dl)

    print(titulo)
    return mapa