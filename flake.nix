{
  description = "Adaptation for rigid control on flexible devices";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flakoboros.follows = "gepetto/flakoboros";
    gazebros2nix.follows = "gepetto/gazebros2nix";
    flake-parts.follows = "gepetto/flake-parts";
    nixpkgs.follows = "gepetto/nixpkgs";
    nix-ros-overlay.follows = "gepetto/nix-ros-overlay";
    systems.follows = "gepetto/systems";
    treefmt-nix.follows = "gepetto/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gepetto.flakeModule
          {
            flakoboros = {
              overrideAttrs.flex-joints = _: _: {
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
              pyOverrideAttrs.flex-joints =
                _: pyFinal:
                (super: {
                  cmakeFlags = super.cmakeFlags ++ [
                    (lib.cmakeBool "BUILD_PYTHON_INTERFACE" true)
                    (lib.cmakeBool "BUILD_STANDALONE_PYTHON_INTERFACE" true)
                  ];
                  propagatedBuildInputs = (super.propagatedBuildInputs or [ ]) ++ [
                    pyFinal.boost
                    pyFinal.eigenpy
                  ];
                  nativeCheckInputs = [
                    pyFinal.pythonImportsCheckHook
                  ];
                });
            };
          }
        ];
      }
    );
}
