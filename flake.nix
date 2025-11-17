{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/ros1-25.05";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
  };
  outputs =
    {
      self,
      nix-ros-overlay,
      nixpkgs,
    }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
          config.permittedInsecurePackages = [
            "freeimage-3.18.0-unstable-2024-04-18"
          ];
        };
      in
      {
        devShells.default = pkgs.mkShell {
          name = "ROS";
          packages = with pkgs; [
            fishPlugins.bass
            colcon
            virt-viewer
            # (pkgs.python312.withPackages (python-pkgs: with python-pkgs; [
            #   influxdb3-python
            #   pyopenssl
            #   service-identity
            #   pymongo
            # ]))            
            (
              with rosPackages.noetic;
              buildEnv {
                paths = [
                  ros-core
                  genmsg
                  rosbridge-suite
                  rosbash
                  turtlebot3-description
                  turtlebot3-teleop
                  turtlebot3-gazebo
                  gazebo-plugins
                  rosbash
                  xacro
                  turtlesim
                  rqt
                  rqt-console
                  rqt-graph
                ];
              }
            )
          ];
          env = {
            ROS_HOSTNAME = "localhost";
            ROS_MASTER_URI = "http://localhost:11311";
            TURTLEBOT3_MODEL = "burger";
            CATKIN_SHELL = "fish";
          };
        };
      }
    );
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
