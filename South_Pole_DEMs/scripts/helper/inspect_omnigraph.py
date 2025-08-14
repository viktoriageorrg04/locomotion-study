import omni.graph.core as og

def inspect_omnigraph():
    graphs = og.get_all_graphs()
    for graph in graphs:
        print("Graph:", graph)
        for node in graph.get_nodes():
            print(" Node:", node.get_path())
            print("  Type:", node.get_type_name())
            print("  Inputs:", node.get_input_names())
            print("  Outputs:", node.get_output_names())