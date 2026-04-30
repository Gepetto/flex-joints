{
  description = "Adaptation for rigid control on flexible devices";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        extraDevPyPackages = [ "flex-joints" ];
        overrideAttrs.flex-joints = {
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
      }
    );
}
