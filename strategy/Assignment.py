import numpy as np

def euclidean(p1, p2):
    if p1 is None or p2 is None:
        return float('inf')
    return np.linalg.norm(np.array(p1) - np.array(p2))

def role_assignment(teammate_positions, formation_positions):

    player_preferences= {}
    roles_preferences= {}

    unmatched_players = [0, 1, 2,3,4]
    current_matches = {0: None, 1: None, 2: None,3: None, 4: None}

#Preferences:
    for i in range(len(teammate_positions)):
        DtoRoles= [euclidean(teammate_positions[i], f) for f in formation_positions]
        player_preferences[i]= np.argsort(DtoRoles).tolist()

    for j in range(len(formation_positions)):
        DtoPlayers= [euclidean(formation_positions[j], p) for p in teammate_positions]
        roles_preferences[j] = np.argsort(DtoPlayers).tolist()

#GaleyShapley
    while unmatched_players:
        player=unmatched_players[0]
        for r in player_preferences[player]:
            if current_matches[r] is None:
                current_matches[r]=player
                unmatched_players.remove(player)
                break
            else:
                potential_Cuck=current_matches[r]
                role_pref=roles_preferences[r]
                if role_pref.index(player)<role_pref.index(potential_Cuck):
                    current_matches[r]=player
                    unmatched_players.remove(player)
                    unmatched_players.append(potential_Cuck)#indeed is a cuck! 
                    break
        else:
            unmatched_players.remove(player)

    stable_marriages={v:k for k,v in current_matches.items()}

    # Input : Locations of all teammate locations and positions
    # Output : Map from unum -> positions
    #-----------------------------------------------------------#

    # Example
    point_preferences = {}
    for i in range(1, 6):
        point_preferences[i] = formation_positions[stable_marriages[i-1]]

    return point_preferences
