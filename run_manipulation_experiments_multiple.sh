# # BCGPLVM (Linear Mapping)
# for i in 1
# do
# 	./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_bcgplvm_linearmapping_seed_2.zip_steps_10_object_configuration_16.npy -i data/manipulation/indices_bcgplvm_linearmapping_seed_2.zip_steps_10_object_configuration_16.npy -n $i
# done

# # BCGPLVM (MLP Mapping)
# for i in 1 2 3 4 5
# do
# 	./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_bcgplvm_mlpmapping_seed_1.zip_steps_10_object_configuration_16.npy -i data/manipulation/indices_bcgplvm_mlpmapping_seed_1.zip_steps_10_object_configuration_16.npy -n $i
# done

# # BCGPLVM (RBF Mapping)
# for i in 1 2 3 4 5
# do
# 	./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_bcgplvm_rbfmapping_seed_1.zip_steps_10_object_configuration_16.npy -i data/manipulation/indices_bcgplvm_rbfmapping_seed_1.zip_steps_10_object_configuration_16.npy -n $i
# done

# # VAE
# for i in 1 2 3 4
# do
# 	./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_vae_seed_4_epochs_200_hiddim_64_kl_1.0_steps_10_object_configuration_16.npy -i data/manipulation/indices_vae_seed_4_epochs_200_hiddim_64_kl_1.0_steps_10_object_configuration_16.npy -n $i
# done

# # VAE 0.1
# for i in 1 2 3 4 5
# do
# 	./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_vae_seed_1_epochs_200_hiddim_64_kl_0.1_steps_10_object_configuration_16.npy -i data/manipulation/indices_vae_seed_1_epochs_200_hiddim_64_kl_0.1_steps_10_object_configuration_16.npy -n $i
# done

# # VAE 10
# for i in 1 2 3 4 5
# do
# 	./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_vae_seed_1_epochs_200_hiddim_64_kl_10.0_steps_10_object_configuration_16.npy -i data/manipulation/indices_vae_seed_1_epochs_200_hiddim_64_kl_10.0_steps_10_object_configuration_16.npy -n $i
# done

# # AE
# for i in 1 2 3 4 5
# do
# 	./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_ae_seed_1_epochs_200_hiddim_64_steps_10_object_configuration_16.npy -i data/manipulation/indices_ae_seed_1_epochs_200_hiddim_64_steps_10_object_configuration_16.npy -n $i
# done

## CVAE
for i in 1 2 3 4 5
do
		./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_cvae_obj_type_size__seed_1_epochs_200_hiddim_64_kl_1.0_steps_10_object_configuration_16.npy -i data/manipulation/indices_cvae_obj_type_size__seed_1_epochs_200_hiddim_64_kl_1.0_steps_10_object_configuration_16.npy -n $i
done

# # # PCA
# for i in 1
# do
# 	./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_pca_steps_10_object_configuration_16.npy -i data/manipulation/indices_pca_steps_10_object_configuration_16.npy -n $i
# done

## Joints
# for i in 1
# do
# 	./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_joint_steps_10_object_configuration_16.npy -i data/manipulation/indices_joint_steps_10_object_configuration_16.npy -n $i
# done
