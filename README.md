<h1 align="left">mystica</h1>

<p align="left">
   <a href="https://github.com/ami-iit/mystica/blob/master/LICENSE"><img src="https://img.shields.io/github/license/ami-iit/mystica" alt="Size" class="center"/></a>
</p>

**Multi bodY SimulaTor maxImal CoordinAtes**

**mystica** is a matlab library to simulate the kinematics of multibody systems.
**mystica** computes the system evolution modeling the problem using a state definition that we define _maximal_.
The _maximal_ representation consists of a set of _non-minimum_ variables 𝐪 complemented by holonomic constraint g(𝐪) = 0.

---

<p align="center">
  <b>:warning: REPOSITORY UNDER DEVELOPMENT :warning:</b>
  <br>The libraries implemented in this repository are still experimental and we cannot guarantee stable API
</p>

---

### Table of content

- [:hammer: Dependencies](#hammer-dependencies)
- [:floppy_disk: Installation](#floppy_disk-installation)
- [:rocket: Usage](#rocket-usage)
- [:gear: Contributing](#gear-contributing)


## :hammer: Dependencies

- [`matlab`](https://mathworks.com/)

Other requisites are:

- [`casadi`](https://web.casadi.org/)
- [`yamlmatlab`](https://github.com/ewiger/yamlmatlab)

Both [`casadi`](https://web.casadi.org/) and [`yamlmatlab`](https://github.com/ewiger/yamlmatlab) are downloaded and configured in section [Installation](#floppy_disk-installation).

## :floppy_disk: Installation

1. Clone the repo:

``` cmd
git clone https://github.com/ami-iit/mystica.git
cd mystica
```

2. Run in **matlab** the function [`install()`](install.m).
``` matlab
install()
```
The function [`install()`](install.m) downloads [`mambaforge`](https://github.com/conda-forge/miniforge#mambaforge). [`mambaforge`](https://github.com/conda-forge/miniforge#mambaforge) is a package manager that downloads and configures our dependencies in conda enviroment called `mystica`.
<details>
    <summary>If you already have <a href="https://github.com/conda-forge/miniforge#mambaforge"><code>mambaforge</code></a></summary>
    If you already have <a href="https://github.com/conda-forge/miniforge#mambaforge"><code>mambaforge</code></a> configured, you can call <a href="install.m"><code>install()</code></a> function defining <code>mambaforge_prefix</code> value:
    <pre><code>install('mambaforge_prefix',&#60;your mambaforge path prefix&#62;)</code></pre>
</details>


⚠️ **Known issue**
For some versions of Ubuntu and Matlab, Matlab is not able to load the required libraries like CasADi (see [issue](https://github.com/ami-iit/mystica/issues/6)). A possible workaround is to launch `matlab` with LD_PRELOAD
```
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 matlab
```

## :rocket: Usage

Before every usage remember to run the autogenerated matlab file called `deps/setup.m` to setup the `MATLABPATH`.

Alternatively, you can launch matlab from terminal after having activated the conda enviroment called `mystica`:
``` cmd
conda activate mystica
matlab
```

Here a snippet of a code to simulate the kinematics of a 4 bar linkage mechanism:

``` matlab
model = mystica.model.getModel4BarLinkage();
stgs = mystica.stgs.getDefaultSettingsSimKinRel(model);
data = mystica.runSimKinRel('model',model,'stgs',stgs,'mBodyPosQuat_0',model.getMBodyPosQuatRestConfiguration,'nameControllerClass','mystica.controller.ExampleKinRel');
mystica.viz.visualizeKinRel('model',model,'data',data,'stgs',stgs)
```

Other examples can be found in the [examples directory](examples).

If you want to go deep into the code, we prepared [this document](docs/nomenclature.md) as an entry point to explain and describe our variables.

## :gear: Contributing

**mystica** is an open-source project, and is thus built with your contributions. We strongly encourage you to open an issue with your feature request. Once the issue has been opened, you can also proceed with a pull-request :rocket:
