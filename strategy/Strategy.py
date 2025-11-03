import math
import numpy as np
from math_ops.Math_Ops import Math_Ops as M
from world.World import World

class Strategy():
    def __init__(self, world):

        self.play_mode = world.play_mode
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
            self.opponent_goal = np.array((15.25, 0.4))
        elif self.ball_2d[1] < -0.05:      
            self.opponent_goal = np.array((15.25, -0.4))
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
        closeToOppGoal= self.BallDistanceToOppGoal()<5.0
        closeToOwnGoal= self.BallDistanceToOwnGoal()<5.0
        state=0

        #Attack strat/ tikki Takka
        if (weAreCloser and not closeToOppGoal):
            state=1
        
        #Shoot
        elif (weAreCloser and closeToOppGoal):
            state =2
        
        #Defend
        elif(not weAreCloser and not closeToOwnGoal):
            state=3

        #defend & park the bus lol
        elif(not weAreCloser and closeToOwnGoal):
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
            # Kickoff pattern: push 6m forward and 6m downward
            return ballPos + np.array((5.0, -5.0))

        # Push the through ball forward and mirror vertically based on current ball y
        offset = np.array((5.0, -5.0)) if ballPos[1] >= 0 else np.array((5.0, 0.0))
        target = ballPos + offset

        if ballPos[0] > 10.0 and ballPos[1] >= 0:
            target[1] = np.clip(target[1], -1.0, 1.0)
            target[0] = min(target[0], 13.0)
        elif ballPos[0] > 7.0 and ballPos[1] >= 0:
            target[1] = np.clip(target[1], -3.0, 3.0)

        elif ballPos[0] > 10.0 and ballPos[1] < 0:
            target[1] = np.clip(target[1], -1.0, 1.0)
            target[0] = min(target[0], 13.0)
        #elif ballPos[0] > 7.0 and ballPos[1] < 0:
            #target[1] = np.clip(target[1], 3.0, 0.0)

        return target

    def makeTriangle(self,agent,world, compact=False, defensive=False):
        scale=3.0
        if compact:
            scale=4.0
        if defensive:
            scale=2.0

        init=[(-scale,-scale), (-scale,0), (scale,scale)]
        idx= (self.player_unum -1)%len(init)
        offset= np.array(init[idx])
        target=self.ball_2d+offset

        return agent.move(target)



    def parkTheBus(self,agent,world):
        
        ys=[-4,-2,0,2,4]
        x=-14 # maybe change to -15 on goal line
        target=(x,ys[self.player_unum -1])

        return agent.move(target)


##Execut All ##   
    
    def Execute(self,agent,world,state=None):

        state=self.GameStates(world)
        opponent_goal = tuple(self.opponent_goal)


        #Attack strat/ tikki Takka
        if state==1:
            is_central_ball= abs(self.ball_2d[0]) < 0.5 and abs(self.ball_2d[1]) < 0.5
            
            if self.player_unum==self.active_player_unum:

                if self.ball_2d[1]<0 and (self.ball_2d[0]>0 and self.ball_2d[0]<7.0):

                    return agent.kickTarget(self,self.mypos,self.ball_2d+(6.0,0.0))    
                
                            
                receiverPos= self.getForwardTeammate()
                if receiverPos is not None:
                    return agent.kickTarget(self,self.mypos,receiverPos+(1.0,1.0))
                
                else: #no one forward


                    if self.play_mode==World.M_OUR_KICKOFF or is_central_ball:
                        return agent.kickTarget(self, self.mypos, (4.0, -6.0))
                    
                    if self.ball_2d[1]<0 and (self.ball_2d[0]>0 and self.ball_2d[0]<7.0):

                        return agent.kickTarget(self,self.mypos,self.ball_2d+(6.0,0.0))

                    return agent.kickTarget(self,self.mypos,opponent_goal)

            elif self.player_unum==self.SecondClosest():
                posTarget=self.findThroughBall(self.ball_2d)
                return agent.move(posTarget)

            else:
                return self.makeTriangle(agent,world)
            

        #Shooting
        elif state==2:  
            if self.player_unum==self.active_player_unum:
                return agent.kickTarget(self,self.mypos,opponent_goal)
            
            else: 
                return self.makeTriangle(agent,world,compact=True)
            
        
        #Defend
        elif state==3:
            if self.player_unum==self.active_player_unum:
                return agent.kick() if np.linalg.norm(np.array(self.ball_2d)-np.array(self.my_head_pos_2d))<0.7 else agent.move(world.ball_abs_pos[:2])

            
            elif self.player_unum==self.SecondClosest():
                return agent.move(world.ball_abs_pos[:2])

            else: 
                return self.makeTriangle(agent,world,defensive=True)
            
        #defend & park the bus lol
        elif state==4:
            if self.player_unum==self.active_player_unum:
                return agent.kick() if np.linalg.norm(np.array(self.ball_2d)-np.array(self.my_head_pos_2d))<0.7 else agent.move(world.ball_abs_pos[:2]) 

            
            elif self.player_unum==self.SecondClosest():
                return agent.move(world.ball_abs_pos[:2])

            else: 
                return self.parkTheBus(agent,world)
            

        else:
            return agent.move(self.my_desired_position)