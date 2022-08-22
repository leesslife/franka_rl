import random
import numpy as np
from collections import defaultdict

class QLearn:
    def __init__(self,actions,epsilon,alpha,gamma):
        self.Q=defaultdict(lambda:np.zeros(3))  # 生成字典，字典的默认值是np.zeros[3]
        self.epsilon=epsilon
        self.alpha=alpha
        self.gamma=gamma
        self.actions=actions
    
    # 用列表的方式更新价值网络
    # trans[state1,action,reward,state2]
    def learn(self,state1,state2,action,reward):
        maxQ_next=np.argmax(self.Q[state2])
        td_target=reward+self.gamma*self.Q[state2][maxQ_next]
        td_delta=td_target-self.Q[state1][action]
        self.Q[state1][action]+=self.alpha*td_delta
    
    def chooseAction(self,state):
        best_action=np.argmax(self.Q[state])
        return best_action

    def printQ(self):
        for i, j in self.Q.items():
            print(i+":")
            print(j)


# if __name__=="__main__":
#     qlearn=QLearn(0,0,0,0)
