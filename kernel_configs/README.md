# Save kernel configurations.

To save the current SDK settings as <config name> use the following command inside spresense/sdk:

tools/mkdefconfig.py -d ../path/to/configs <config name>

To use a defconfig in a different path, in this example ../path/to/configs use:

tools/config.py -d ../path/to/configs <config name>

### piccole note / small tips

- It's always a good habit to enable ONLY what you need of.
- Don't worry about cleaning, we'll handle it 'someday'.