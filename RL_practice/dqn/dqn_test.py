
import gym
from gym.wrappers import Monitor
import itertools
import numpy as np
import os
import random
import sys
import tensorflow as tf

from dqn_class import StateProcessor, Estimator

def make_greedy_policy(estimator, nA):
    def policy_fn(sess, observation, epsilon):
        A = np.zeros(nA, dtype=float) 
        q_values = estimator.predict(sess, np.expand_dims(observation, 0))[0]
        best_action = np.argmax(q_values)
        A[best_action] += 1.0
        return A
    return policy_fn


if __name__ == '__main__':
    env = gym.envs.make("Breakout-v0")
    VALID_ACTIONS = [0, 1, 2, 3]
    tf.reset_default_graph()

    # final result save path
    result_file = os.path.abspath("./dqn_save_20000/model.ckpt")

    # Create a glboal step variable
    global_step = tf.Variable(0, name='global_step', trainable=False)
        
    # Create estimators
    q_estimator = Estimator(scope="q")

    # State processor
    state_processor = StateProcessor()

    # Run it!
    with tf.Session() as sess:
        result_loader = tf.train.Saver()
        print("Loading model ...\n")
        result_loader.restore(sess, result_file)
        print("model loaded ...\n")
        # test
        ob = env.reset() 
        ob = env.reset() 
        ob = env.reset() 
        ob = state_processor.process(sess, ob) 
        ob = np.stack([ob] * 4, axis=2) 
        rewards = 0
        
        start_count = 0
        last_action = 0

        while True:
            # The policy we're following
            policy = make_greedy_policy(q_estimator, len(VALID_ACTIONS))
            action_probs = policy(sess, ob, 0)
            action = np.random.choice(np.arange(len(action_probs)), p=action_probs)
            next_ob, reward, done, _ = env.step(action)
            if last_action == action and reward == 0:
                start_count += 1
            else:
                start_count = 0
            if start_count > 5:
                next_ob, reward, done, _ = env.step(1)

            rewards += reward

            print "action is", action, "reward is", reward
            next_ob = state_processor.process(sess, next_ob)
            next_ob = np.append(ob[:,:,1:], np.expand_dims(next_ob, 2), axis=2)
            env.render()
            if done:

                print "Episode done, reward is", rewards
                rewards = 0
                env.reset()
            ob = next_ob
            last_action = action
