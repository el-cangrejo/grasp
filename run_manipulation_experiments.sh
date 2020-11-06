# # BCGPLVM (Linear Mapping)
# ./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_bcgplvm_linearmapping_seed_1.zip_steps_10_object_configuration_16.npy -i data/manipulation/indices_bcgplvm_linearmapping_seed_1.zip_steps_10_object_configuration_16.npy

# # BCGPLVM (MLP Mapping)
# ./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_bcgplvm_mlpmapping_seed_1.zip_steps_10_object_configuration_16.npy -i data/manipulation/indices_bcgplvm_mlpmapping_seed_1.zip_steps_10_object_configuration_16.npy

# # BCGPLVM (RBF Mapping)
# ./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_bcgplvm_rbfmapping_seed_1.zip_steps_10_object_configuration_16.npy -i data/manipulation/indices_bcgplvm_rbfmapping_seed_1.zip_steps_10_object_configuration_16.npy

# # VAE
# ./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_vae_seed_1_epochs_200_hiddim_64_kl_1.0_steps_10_object_configuration_16.npy -i data/manipulation/indices_vae_seed_1_epochs_200_hiddim_64_kl_1.0_steps_10_object_configuration_16.npy

# # AE
# ./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_ae_seed_1_epochs_200_hiddim_64_steps_10_object_configuration_16.npy -i data/manipulation/indices_ae_seed_1_epochs_200_hiddim_64_steps_10_object_configuration_16.npy

# # CVAE
# ./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_cvae_obj_type_size__seed_1_epochs_100_hiddim_64_kl_1.0_steps_10_object_configuration_16.npy -i data/manipulation/indices_cvae_obj_type_size__seed_1_epochs_100_hiddim_64_kl_1.0_steps_10_object_configuration_16.npy

# # PCA
# ./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_pca_steps_10_object_configuration_16.npy -i data/manipulation/indices_pca_steps_10_object_configuration_16.npy

Joints
./build/bin/manipulation -c cfg/robots.yml -r shadow -t data/manipulation/trajectories_joint_steps_10_object_configuration_16.npy -i data/manipulation/indices_joint_steps_10_object_configuration_16.npy
