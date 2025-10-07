{
  description = "Adaptation for rigid control on flexible devices";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    nixpkgs.follows = "gepetto/nixpkgs";
    nix-ros-overlay.follows = "gepetto/nix-ros-overlay";
    systems.follows = "gepetto/systems";
    treefmt-nix.follows = "gepetto/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import inputs.systems;
      imports = [ inputs.gepetto.flakeModule ];
      perSystem =
        {
          lib,
          pkgs,
          self',
          ...
        }:
        {
          packages = {
            default = self'.packages.py-flex-joints;
            flex-joints = pkgs.flex-joints.overrideAttrs {
              src = lib.fileset.toSource {
                root = ./.;
                fileset = lib.fileset.unions [
                  ./include
                  ./src
                  ./python
                  ./tests
                  ./CMakeLists.txt
                  ./package.xml
                ];
              };
            };
            py-flex-joints = pkgs.python3Packages.toPythonModule (
              self'.packages.flex-joints.overrideAttrs (super: {
                pname = "py-${super.pname}";
                cmakeFlags = super.cmakeFlags ++ [
                  (lib.cmakeBool "BUILD_PYTHON_INTERFACE" true)
                  (lib.cmakeBool "BUILD_STANDALONE_PYTHON_INTERFACE" true)
                ];
                propagatedBuildInputs = (super.propagatedBuildInputs or [ ]) ++ [
                  self'.packages.flex-joints
                  pkgs.python3Packages.boost
                  pkgs.python3Packages.eigenpy
                ];
                nativeCheckInputs = [
                  pkgs.python3Packages.pythonImportsCheckHook
                ];
              })
            );
          };
        };
    };
}
