import math
import numpy as np
from math_ops.Math_Ops import Math_Ops as M
from world.World import World

class Strategy():
    SET_PIECE_PLAY_MODES = {
        World.M_OUR_KICKOFF,
        World.M_OUR_FREE_KICK,
        World.M_OUR_DIR_FREE_KICK,
        World.M_OUR_CORNER_KICK,
    }

    set_piece_last_kicker = None
    set_piece_waiting_for_other_touch = False
    set_piece_lock_start_ms = 0
    pending_set_piece_kicker = None
    pending_set_piece_ball_pos = None
    pending_set_piece_time_ms = 0
    last_play_mode = None
    FIELD_X_BOUNDS = (-15.0, 15.0)
    FIELD_Y_BOUNDS = (-10.0, 10.0)

    def __init__(self, world):

        self.play_mode = world.play_mode
        self.time_ms = world.time_local_ms
        self.robot_model = world.robot  
        self.my_head_pos_2d = self.robot_model.loc_head_position[:2]
        self.player_unum = self.robot_model.unum
        self.mypos = (world.teammates[self.player_unum-1].state_abs_pos[0],world.teammates[self.player_unum-1].state_abs_pos[1])
       
        self.side = 1
        if world.team_side_is_left:
            self.side = 0

        self.teammate_positions = [teammate.state_abs_pos[:2] if teammate.state_abs_pos is not None 
                                    else None
                                    for teammate in world.teammates
                                    ]
        
        self.opponent_positions = [opponent.state_abs_pos[:2] if opponent.state_abs_pos is not None 
                                    else None
                                    for opponent in world.opponents
                                    ]



        

        self.team_dist_to_ball = None
        self.team_dist_to_oppGoal = None
        self.opp_dist_to_ball = None

        self.prev_important_positions_and_values = None
        self.curr_important_positions_and_values = None
        self.point_preferences = None
        self.combined_threat_and_definedPositions = None


        self.my_ori = self.robot_model.imu_torso_orientation
        self.ball_2d = world.ball_abs_pos[:2]
        self.ball_vec = self.ball_2d - self.my_head_pos_2d
        self.ball_dir = M.vector_angle(self.ball_vec)
        self.ball_dist = np.linalg.norm(self.ball_vec)
        self.ball_sq_dist = self.ball_dist * self.ball_dist # for faster comparisons
        self.ball_speed = np.linalg.norm(world.get_ball_abs_vel(6)[:2])

        if self.ball_2d[1] > 0.05:
            self.opponent_goal = np.array((15.25, 0.45))
        elif self.ball_2d[1] < -0.05:      
            self.opponent_goal = np.array((15.25, -0.45))
        else:                            
            self.opponent_goal = np.array((15.25, 0.0))

        self.goal_dir = M.target_abs_angle(self.ball_2d, self.opponent_goal)

        self.PM_GROUP = world.play_mode_group

        self.slow_ball_pos = world.get_predicted_ball_pos(0.5) # predicted future 2D ball position when ball speed <= 0.5 m/s

        # list of squared distances between teammates (including self) and slow ball (sq distance is set to 1000 in some conditions)
        self.teammates_ball_sq_dist = [np.sum((p.state_abs_pos[:2] - self.slow_ball_pos) ** 2)  # squared distance between teammate and ball
                                  if p.state_last_update != 0 and (world.time_local_ms - p.state_last_update <= 360 or p.is_self) and not p.state_fallen
                                  else 1000 # force large distance if teammate does not exist, or its state info is not recent (360 ms), or it has fallen
                                  for p in world.teammates ]

        # list of squared distances between opponents and slow ball (sq distance is set to 1000 in some conditions)
        self.opponents_ball_sq_dist = [np.sum((p.state_abs_pos[:2] - self.slow_ball_pos) ** 2)  # squared distance between teammate and ball
                                  if p.state_last_update != 0 and world.time_local_ms - p.state_last_update <= 360 and not p.state_fallen
                                  else 1000 # force large distance if opponent does not exist, or its state info is not recent (360 ms), or it has fallen
                                  for p in world.opponents ]

        self.min_teammate_ball_sq_dist = min(self.teammates_ball_sq_dist)
        self.min_teammate_ball_dist = math.sqrt(self.min_teammate_ball_sq_dist)   # distance between ball and closest teammate
        self.min_opponent_ball_dist = math.sqrt(min(self.opponents_ball_sq_dist)) # distance between ball and closest opponent

        self.active_player_unum = self.teammates_ball_sq_dist.index(self.min_teammate_ball_sq_dist) + 1

        self.my_desired_position = self.mypos
        self.my_desired_orientation = self.ball_dir
        self._update_set_piece_state()

    def _clamp_to_field(self, target):
        """Keep 2D target inside the playable pitch before issuing movement commands."""
        x = float(np.clip(target[0], *self.FIELD_X_BOUNDS))
        y = float(np.clip(target[1], *self.FIELD_Y_BOUNDS))
        return np.array((x, y))


    def GenerateTeamToTargetDistanceArray(self, target, world):
        for teammate in world.teammates:
            pass
        

    def IsFormationReady(self, point_preferences):
        
        is_formation_ready = True
        for i in range(1, 6):
            if i != self.active_player_unum: 
                teammate_pos = self.teammate_positions[i-1]

                if not teammate_pos is None:

                    distance = np.sum((teammate_pos - point_preferences[i]) **2)
                    if(distance > 0.3):
                        is_formation_ready = False

        return is_formation_ready

    def GetDirectionRelativeToMyPositionAndTarget(self,target):
        target_vec = target - self.my_head_pos_2d
        target_dir = M.vector_angle(target_vec)

        return target_dir

    def _update_set_piece_state(self):
        now = self.time_ms

        if Strategy.last_play_mode != self.play_mode:
            if self.play_mode in Strategy.SET_PIECE_PLAY_MODES:
                Strategy.set_piece_waiting_for_other_touch = False
                Strategy.set_piece_last_kicker = None
                Strategy.set_piece_lock_start_ms = 0
            Strategy.pending_set_piece_kicker = None
            Strategy.pending_set_piece_ball_pos = None
            Strategy.pending_set_piece_time_ms = 0
            Strategy.last_play_mode = self.play_mode

        if Strategy.pending_set_piece_kicker is not None:
            ball_moved = False
            if Strategy.pending_set_piece_ball_pos is not None:
                ball_moved = np.linalg.norm(self.ball_2d - Strategy.pending_set_piece_ball_pos) > 0.15
            ball_moved = ball_moved or self.ball_speed > 0.25
            if ball_moved:
                Strategy.set_piece_waiting_for_other_touch = True
                Strategy.set_piece_last_kicker = Strategy.pending_set_piece_kicker
                Strategy.set_piece_lock_start_ms = now
                Strategy.pending_set_piece_kicker = None
                Strategy.pending_set_piece_ball_pos = None
                Strategy.pending_set_piece_time_ms = 0
            elif Strategy.pending_set_piece_time_ms and now - Strategy.pending_set_piece_time_ms > 3000:
                Strategy.pending_set_piece_kicker = None
                Strategy.pending_set_piece_ball_pos = None
                Strategy.pending_set_piece_time_ms = 0

        if Strategy.set_piece_waiting_for_other_touch and Strategy.set_piece_last_kicker is not None:
            other_teammate_close = (self.active_player_unum != Strategy.set_piece_last_kicker and 
                                    self.min_teammate_ball_dist < 0.6)
            opponent_close = self.min_opponent_ball_dist < 0.6
            timed_out = Strategy.set_piece_lock_start_ms and (now - Strategy.set_piece_lock_start_ms > 5000)
            if other_teammate_close or opponent_close or timed_out:
                Strategy.set_piece_waiting_for_other_touch = False
                Strategy.set_piece_last_kicker = None
                Strategy.set_piece_lock_start_ms = 0

    def _is_set_piece_play_mode(self):
        return self.play_mode in Strategy.SET_PIECE_PLAY_MODES

    def _register_set_piece_attempt(self):
        if self._is_set_piece_play_mode() and not Strategy.set_piece_waiting_for_other_touch:
            if Strategy.pending_set_piece_kicker is None or Strategy.pending_set_piece_kicker == self.player_unum:
                Strategy.pending_set_piece_kicker = self.player_unum
                Strategy.pending_set_piece_ball_pos = np.array(self.ball_2d, copy=True)
                Strategy.pending_set_piece_time_ms = self.time_ms

    def _is_kick_blocked(self):
        return (Strategy.set_piece_waiting_for_other_touch and
                Strategy.set_piece_last_kicker == self.player_unum)

    def _hold_after_set_piece(self, agent):
        offset_x = -0.6 if self.side == 0 else 0.6
        hold_target = tuple(self.ball_2d + np.array((offset_x, 0.0)))
        return agent.move(hold_target)

    def _attempt_kick(self, agent, target):
        if self._is_kick_blocked():
            return self._hold_after_set_piece(agent)
        target_point = tuple(self._clamp_to_field(np.array(target)))
        result = agent.kickTarget(self, self.mypos, target_point)
        self._register_set_piece_attempt()
        return result
    
######################################################################################################

## Helpers ##

    def BallDistanceToOppGoal(self):
        return np.linalg.norm(self.ball_2d - self.opponent_goal)
    
    def BallDistanceToOwnGoal(self):
        own_goal= np.array((-15.0,0))
        return np.linalg.norm(self.ball_2d-own_goal)
    
    #4 states#
    def GameStates(self,world):

        weAreCloser=self.min_teammate_ball_dist<=self.min_opponent_ball_dist
        closeToOppGoal= self.BallDistanceToOppGoal()<6.5
        closeToOwnGoal= self.BallDistanceToOwnGoal()<6.5
        state=0

        #Attack strat/ tikki Takka
        if (weAreCloser and not closeToOppGoal):
            state=1
        
        #Shoot
        elif (closeToOppGoal):
            state=2
        
        #Defend
        elif(not weAreCloser and not closeToOwnGoal):
            state=3

        #defend & park the bus lol
        elif(closeToOwnGoal):
            state=4

        else:
            state=0

        #track states while viewing; s check logs
        print(f"GameState={state},BallPos={self.ball_2d},OurMinDist={self.min_teammate_ball_dist:.2f},OppMinDist={self.min_opponent_ball_dist:.2f}")

        return state
    

    
    def getForwardTeammate(self):

        options=[]
        for p in self.teammate_positions:
            if p is not None and ((self.side==0 and p[0]>self.mypos[0]) or (self.side!=0 and p[0]<self.mypos[0])):
                options.append(p)
        
        if len(options)==0:
            return None
        
        distances=[np.linalg.norm(np.array(p)-np.array(self.mypos)) for p in options]

        if self.side==0:
            return max(options, key=lambda p: p[0])
        else:
            return min(options, key=lambda p: p[0]) 

        
    def SecondClosest(self):
        sortedOptions= np.argsort(self.teammates_ball_sq_dist)

        if len(sortedOptions)<2:
            return None
        return sortedOptions[1]+1
    
    def findThroughBall(self,ballPos):

        is_central_ball=abs(ballPos[0])<0.5 and abs(ballPos[1])<0.5
        if self.play_mode in (World.M_OUR_KICKOFF, World.M_THEIR_KICKOFF) or is_central_ball:
            # Kickoff pattern: push 4m forward and 6m downward
            return self._clamp_to_field(ballPos + np.array((4.0, -6.0)))

        # Push the through ball forward and mirror vertically based on current ball y
        offset = np.array((0.0,0.0))

        if ballPos[1] >= 0:
            offset = np.array((5.0, -5.0))
        elif ballPos[1] < 0:
            offset = np.array((5.0, 0.0))

        target = ballPos + offset

        if ballPos[1] >= 0:
            if ballPos[0] > 10.0:
                lateral_floor = min(ballPos[1], -1.0)
                target[1] = min(target[1], lateral_floor)
                target[0] = min(target[0], 13.0)
            elif ballPos[0] > 7.0:
                lateral_floor = min(ballPos[1], -5.0)
                target[1] = min(target[1], lateral_floor)
                target[0] = min(target[0], 11.0)
            elif ballPos[0] >= 0.0:
                target[1] = max(target[1], -4.0)

        elif ballPos[0] > 10.0 and ballPos[1] < 0:
            target[1] = np.clip(target[1], -1.0, 1.0)
            target[0] = min(target[0], 13.0)
        elif ballPos[0] > 5.0 and ballPos[1] < 0:
            lateral_floor = max(ballPos[1], -5.0)
            target[1] = max(target[1], lateral_floor)
            target[0] = min(target[0], 12.0)


        return self._clamp_to_field(target)

    def makeTriangle(self,agent,world, compact=False, defensive=False):
        scale=3.0
        if compact:
            scale=4.0
        if defensive:
            scale=2.0

        init=[(-scale,-scale), (-scale,0), (scale,scale)]
        idx= (self.player_unum -1)%len(init)
        offset= np.array(init[idx])
        target=self._clamp_to_field(self.ball_2d+offset)

        return agent.move(target)



    def parkTheBus(self,agent,world):
        
        ys=[-4,-2,0,2,4]
        x=-14 # maybe change to -15 on goal line
        target=self._clamp_to_field(np.array((x,ys[self.player_unum -1])))

        return agent.move(target)


##Execut All ##   
    
    def Execute(self,agent,world,state=None):

        state=self.GameStates(world)
        opponent_goal = tuple(self.opponent_goal)


        #Attack strat/ tikki Takka
        if state==1:
            is_central_ball= abs(self.ball_2d[0]) < 0.5 and abs(self.ball_2d[1]) < 0.5
            through_target = self.findThroughBall(self.ball_2d)
            receiverPos= self.getForwardTeammate()

            #if I am closest to ball
            if self.player_unum==self.active_player_unum:

                #and if ball is behind x = 13
                if self.ball_2d[0]<=13 and self.ball_2d[0]>=-0.5: 
                    return self._attempt_kick(agent, through_target)
                
                if self.ball_2d[0]<-0.5: 
                    target_choice = receiverPos if receiverPos is not None else through_target
                    return self._attempt_kick(agent, target_choice)    
                
                # and if there is a reciever ahead of me, closest to the opp goals,
                # and ball is ahead of x = 10    
                # (we want more accurate passes closer to the goal)            
                elif receiverPos is not None and self.ball_2d[0]>13: 
                    return self._attempt_kick(agent, receiverPos+(0.5,0.5))
                
                # and no one ahead of me
                else:
                    #and its kick off
                    if self.play_mode==World.M_OUR_KICKOFF or is_central_ball:
                        return self._attempt_kick(agent, through_target)

                    #shoot
                    return self._attempt_kick(agent, opponent_goal)

            elif self.player_unum==self.SecondClosest():
                posTarget=self.findThroughBall(self.ball_2d)
                return agent.move(self._clamp_to_field(posTarget))

            else:
                return self.makeTriangle(agent,world)
            

        #Shooting
        elif state==2:  
            if self.player_unum==self.active_player_unum:
                if self.ball_2d[0]>14.5 and abs(self.ball_2d[1])>1.0:
                    return self._attempt_kick(agent, (14.5,0.0))
                
                return self._attempt_kick(agent, opponent_goal)
            
            else: 
                return self.makeTriangle(agent,world,compact=True)
            
        
        #Defend
        elif state==3:
            if self.player_unum==self.active_player_unum:
                close_to_ball = np.linalg.norm(np.array(self.ball_2d)-np.array(self.my_head_pos_2d))<0.7
                if close_to_ball:
                    if self._is_kick_blocked():
                        return self._hold_after_set_piece(agent)
                    self._register_set_piece_attempt()
                    return agent.kick()
                return agent.move(self._clamp_to_field(world.ball_abs_pos[:2]))

            
            elif self.player_unum==self.SecondClosest():
                return agent.move(self._clamp_to_field(world.ball_abs_pos[:2]))

            else: 
                return self.makeTriangle(agent,world,defensive=True)
            
        #defend & park the bus lol
        elif state==4:
            if self.player_unum==self.active_player_unum:
                close_to_ball = np.linalg.norm(np.array(self.ball_2d)-np.array(self.my_head_pos_2d))<0.7
                if close_to_ball:
                    if self._is_kick_blocked():
                        return self._hold_after_set_piece(agent)
                    self._register_set_piece_attempt()
                    return agent.kick()
                return agent.move(self._clamp_to_field(world.ball_abs_pos[:2])) 

            
            elif self.player_unum==self.SecondClosest():
                return agent.move(self._clamp_to_field(world.ball_abs_pos[:2]))

            else: 
                return self.parkTheBus(agent,world)
            

        else:
            return agent.move(self._clamp_to_field(self.my_desired_position))
