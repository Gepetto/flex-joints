{
  description = "Adaptation for rigid control on flexible devices";

  inputs = {
    # Use MaximilienNaveau/nix until https://github.com/Gepetto/nix/pull/110 is merged.
    gepetto.url = "github:MaximilienNaveau/nix/topic/mnaveau/add-flex-joints";
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
            default = self'.packages.flex-joints;
            flex-joints = pkgs.python3Packages.flex-joints.overrideAttrs {
              checkInputs = [ pkgs.doctest ];
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
          };
        };
    };
}
