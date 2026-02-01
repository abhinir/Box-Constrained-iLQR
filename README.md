<div align="center">
  <h1>Safe Optimal Control using Log Barrier Constrained iLQR</h1>
  <p>
    <strong>Abhijeet<sup>1</sup></strong>, and <strong>Suman Chakravorty<sup>1</sup></strong>
  </p>
  <p>
    <sup>1</sup>Texas A&M University, College Station, Texas, U.S.A.
  </p>
</div>

---

<div align="center">
<h2>
    <a href="wafr2026_paper.pdf">Paper (PDF)</a>
    &nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
    <a href="https://github.com/abhinir/Box-iLQR/tree/main/Code">Code</a>
    &nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
    <a href="#video-demo">Video</a>
    &nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp;
    <a href="#snapshots">Snapshots</a>
</h2>
</div>

---




## Abstract
This paper presents a constrained iterative Linear Quadratic Regulator (iLQR) framework for nonlinear optimal control problems with box constraints on both states and control inputs. We incorporate logarithmic barrier functions into the stage cost to enforce box constraints (upper and lower bounds on variables), yielding a smooth interior-point formulation that integrates seamlessly with the standard iLQR backward–forward pass. The Hessian contributions from the log barriers are positive definite, preserving and enhancing the positive definiteness of
the quadratic approximations in iLQR and providing an intrinsic regularization effect that improves numerical stability and convergence. Moreover, since the negative logarithm is convex, the addition of log barrier terms preserves convexity if the cost is already convex. We further analyze how the barrier-augmented iLQR naturally adapts feedback gains near constraint boundaries. In particular, at convergence, the feedback terms associated with saturated control channels go to zero, recovering a purely feedforward behavior whenever control is saturated. Numerical
examples on constrained nonlinear control problems demonstrate that the proposed method reliably respects box constraints and maintains favorable convergence properties.

<!-- You can add a 'teaser' image from your paper here! -->
<!-- First, upload the image (e.g., teaser.png) to your repo just like the PDF. -->
<!-- Then, uncomment the line below and change the filename. -->
<!-- <img src="teaser.png" alt="Teaser Image" style="width:100%; max-width: 800px; display: block; margin-left: auto; margin-right: auto;"/> -->
<!-- *Fig. 1: A brief, one-sentence caption for your teaser image.* -->

---

<a name="video-demo"></a>
## Swing-up Tasks for Pendulum, Cart-pole and Acrobot
<div align="center">
<!-- The 'width: 400' and 'table-layout: fixed' are the key fixes -->
<table style="width: 100%; border: none; table-layout: fixed;">
  <!-- ROW 1 -->
  <tr style="border: none;">
    <!-- Added 'text-align: center' for a cleaner look -->
    <td style="padding: 10px; border: none; text-align: center;">
      <iframe width="400" height="240" src="https://www.youtube.com/embed/kRjgkPTZJys" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
      <br>
      <em>Pendulum swing-up without any constraints.</em>
    </td>
    <td style="padding: 10px; border: none; text-align: center;">
      <iframe width="400" height="240" src="https://www.youtube.com/embed/GFmePDyHEG4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
      <br>
      <em>Pendulum swing-up with actuation constraints: -1 &lt; u &lt; 1.</em>
    </td>
  </tr>
  <!-- ROW 2 -->
  <tr style="border: none;">
    <td style="padding: 10px; border: none; text-align: center;">
      <iframe width="400" height="240" src="https://www.youtube.com/embed/mD84pGqbik4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
      <br>
      <em>Cartpole swing-up without any constraints.</em>
    </td>
    <td style="padding: 10px; border: none; text-align: center;">
      <iframe width="400" height="240" src="https://www.youtube.com/embed/QGBJvQMlQJ8" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
      <br>
      <em>Cart-pole swing-up with state and control constraints (-2 &lt; u &lt; 2 and -0.2 &lt; x_1 &lt; 0.2).</em>
    </td>
  </tr>
  <!-- ROW 3 -->
  <tr style="border: none;">
    <td style="padding: 10px; border: none; text-align: center;">
      <iframe width="400" height="240" src="https://www.youtube.com/embed/2Zl8muy1ZYc" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
      <br>
      <em>Acrobot swing-up without any constraints.</em>
    </td>
    <td style="padding: 10px; border: none; text-align: center;">
      <iframe width="400" height="240" src="https://www.youtube.com/embed/4PJ0NKPSZlY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
      <br>
      <em>Acrobot swing-up with control constraints: -5 &lt; u &lt; 5.</em>
    </td>
  </tr>
</table>
</div>

---


<a name="snapshots"></a>
## Trajectory comparison using snapshots

<div align="center">
  <img src="pendulum_comp_snapshots.png" width="800" alt="A descriptive caption for my image">
  <br>
  <em>Fig. 1: Time-lapse visualization of the pendulum swing-up task, comparing the unconstrained trajectory (light blue) against the trajectory with both state and control constraints (red).</em>
</div>

---

<div align="center">
  <img src="cartpole_comp_snapshots.png" width="800" alt="A descriptive caption for my image">
  <br>
  <em>Fig. 2: Time-lapse visualization of the cart-pole swing-up task, comparing the unconstrained trajectory (light blue) against the trajectory with both state and control constraints (red).</em>
</div>

---

<div align="center">
  <img src="acrobot_comp_snapshots.png" width="800" alt="A descriptive caption for my image">
  <br>
  <em>Fig. 3: Time-lapse visualization of the acrobot swing-up task, comparing the unconstrained trajectory (light blue) against the trajectory with both state and control constraints (red).</em>
</div>

---

## Citation
If you use our work, please cite our paper:


@inproceedings{Abhijeet,
  title   = {Title of Your Paper},
  author  = {Your, Name and Co-author, Name},
  booktitle = {alpha-beta-gamma},
  year    = {2026}
}

