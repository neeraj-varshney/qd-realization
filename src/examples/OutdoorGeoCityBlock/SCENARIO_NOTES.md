# OutdoorGeoCityBlock Scenario Notes

This scenario demonstrates outdoor AMF/XML generation from latitude/longitude using open-access OSM Overpass API.

## Key Inputs

- `latitude`, `longitude`, and optional `outdoorQueryRadius` are set in `Input/paraCfgCurrent.txt`.
- `materialLibraryPath` is set to `material_libraries/materialLibraryCityBlock.csv`.
- Material mapping in generated AMF follows CityBlock conventions:
  - material id `1` => `Buildings`
  - material id `2` => `Ground`

## Runtime Behavior

At runtime, `parameterCfg` generates:

- `Input/OutdoorGeneratedFromLatLon.amf`

from the configured coordinates, then uses it as `environmentFileName`.

## Backward Compatibility

If `latitude`/`longitude` are not present in a scenario config, legacy file-based environment loading is unchanged.
