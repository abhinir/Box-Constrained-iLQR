# Safe Optimal Control using Log Barrier Constrained iLQR

**Abhijeet<sup>1</sup>**, and **Suman Chakravorty<sup>1</sup>**

<sup>1</sup>Texas A&M University, College Station, Texas, US

---

### [Paper (PDF)](wafr2026_paper.pdf) &nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp; [Code](https://github.com/abhinir/Box-Constrained-iLQR) &nbsp;&nbsp;&nbsp;|&nbsp;&nbsp;&nbsp; [Video](#video-demo)

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
## Video Demo

<!-- Paste your YouTube embed code here. Centered for a clean look. -->
<div align="center">
  <iframe width="800" height="450" src="https://youtu.be/4PJ0NKPSZlY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</div>

---

## Citation
If you use our work, please cite our paper:

```bibtex
@inproceedings{YOURNAME2026wafr,
  title   = {Title of Your Paper},
  author  = {Your, Name and Co-author, Name},
  booktitle = {Proceedings of the Workshop on the Algorithmic Foundations of Robotics (WAFR)},
  year    = {2026}
}
