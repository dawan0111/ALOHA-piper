import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from act_environment.envs.act_environment import DualArmPiperEnv
import threading

class EnvMainNode(Node):
    def __init__(self):
        super().__init__("dual_arm_piper_env_node")
        self.env = DualArmPiperEnv(node=self)
        self.timestep = self.env.reset()
        self._running = True
        self._step_thread = threading.Thread(target=self._run_loop)
        self._step_thread.start()

    def _run_loop(self):
        set_seed(1000)
        ckpt_dir = config['ckpt_dir']
        state_dim = config['state_dim']
        real_robot = config['real_robot']
        policy_class = config['policy_class']
        camera_names = config['camera_names']
        max_timesteps = config['episode_len']
        task_name = config['task_name']
        temporal_agg = config['temporal_agg']
        onscreen_cam = 'angle'

        # load policy and stats
        ckpt_path = os.path.join(ckpt_dir, ckpt_name)
        policy = make_policy(policy_class, policy_config)
        loading_status = policy.load_state_dict(torch.load(ckpt_path))
        print(loading_status)
        policy.cuda()
        policy.eval()
        print(f'Loaded: {ckpt_path}')
        stats_path = os.path.join(ckpt_dir, f'dataset_stats.pkl')
        with open(stats_path, 'rb') as f:
            stats = pickle.load(f)

        pre_process = lambda s_qpos: (s_qpos - stats['qpos_mean']) / stats['qpos_std']
        post_process = lambda a: a * stats['action_std'] + stats['action_mean']

        # load environment
        if real_robot:
            from aloha_scripts.robot_utils import move_grippers # requires aloha
            from aloha_scripts.real_env import make_real_env # requires aloha
            env = make_real_env(init_node=True)
            env_max_reward = 0
        else:
            from sim_env import make_sim_env
            env = make_sim_env(task_name)
            env_max_reward = env.task.max_reward

        query_frequency = policy_config['num_queries']
        if temporal_agg:
            query_frequency = 1
            num_queries = policy_config['num_queries']

        max_timesteps = int(max_timesteps * 1) # may increase for real-world tasks

        num_rollouts = 50
        episode_returns = []
        highest_rewards = []
        for rollout_id in range(num_rollouts):
            rollout_id += 0
            ### set task
            if 'sim_transfer_cube' in task_name:
                BOX_POSE[0] = sample_box_pose() # used in sim reset
            elif 'sim_insertion' in task_name:
                BOX_POSE[0] = np.concatenate(sample_insertion_pose()) # used in sim reset

            ts = env.reset()

            ### onscreen render
            if onscreen_render:
                ax = plt.subplot()
                plt_img = ax.imshow(env._physics.render(height=480, width=640, camera_id=onscreen_cam))
                plt.ion()

            ### evaluation loop
            if temporal_agg:
                all_time_actions = torch.zeros([max_timesteps, max_timesteps+num_queries, state_dim]).cuda()

            qpos_history = torch.zeros((1, max_timesteps, state_dim)).cuda()
            image_list = [] # for visualization
            qpos_list = []
            target_qpos_list = []
            rewards = []
            with torch.inference_mode():
                for t in range(max_timesteps):
                    ### update onscreen render and wait for DT
                    if onscreen_render:
                        image = env._physics.render(height=480, width=640, camera_id=onscreen_cam)
                        plt_img.set_data(image)
                        plt.pause(DT)

                    ### process previous timestep to get qpos and image_list
                    obs = ts.observation
                    if 'images' in obs:
                        image_list.append(obs['images'])
                    else:
                        image_list.append({'main': obs['image']})
                    qpos_numpy = np.array(obs['qpos'])
                    qpos = pre_process(qpos_numpy)
                    qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)
                    qpos_history[:, t] = qpos
                    curr_image = get_image(ts, camera_names)

                    ### query policy
                    if config['policy_class'] == "ACT":
                        if t % query_frequency == 0:
                            all_actions = policy(qpos, curr_image)
                        if temporal_agg:
                            all_time_actions[[t], t:t+num_queries] = all_actions
                            actions_for_curr_step = all_time_actions[:, t]
                            actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                            actions_for_curr_step = actions_for_curr_step[actions_populated]
                            k = 0.01
                            exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                            exp_weights = exp_weights / exp_weights.sum()
                            exp_weights = torch.from_numpy(exp_weights).cuda().unsqueeze(dim=1)
                            raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)
                        else:
                            raw_action = all_actions[:, t % query_frequency]
                    elif config['policy_class'] == "CNNMLP":
                        raw_action = policy(qpos, curr_image)
                    else:
                        raise NotImplementedError

                    ### post-process actions
                    raw_action = raw_action.squeeze(0).cpu().numpy()
                    action = post_process(raw_action)
                    target_qpos = action

                    ### step the environment
                    ts = env.step(target_qpos)

                    ### for visualization
                    qpos_list.append(qpos_numpy)
                    target_qpos_list.append(target_qpos)
                    rewards.append(ts.reward)

                plt.close()
            if real_robot:
                move_grippers([env.puppet_bot_left, env.puppet_bot_right], [PUPPET_GRIPPER_JOINT_OPEN] * 2, move_time=0.5)  # open
                pass

        
        while rclpy.ok() and self._running:
            try:
                action = [0.0] * self.env.robot.action_size
                self.timestep = self.env.step(action)
                self.get_logger().info(f"[STEP] {self.timestep.step_type}, Reward: {self.timestep.reward}")
            except Exception as e:
                self.get_logger().error(f"[STEP ERROR] {e}")

    def get_env(self):
        return self.env

    def stop(self):
        self._running = False
        self._step_thread.join()

if __name__ == "__main__":
    rclpy.init()
    node = EnvMainNode()
    env = DualArmPiperEnv(node=node)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down environment node...")