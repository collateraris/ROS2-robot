import pytest
import numpy as np
torch=pytest.importorskip("torch")
from learning_nav.models import WorldModel,TinyVLA,masked_scan_mse


def test_world_uses_action_and_horizon():
    torch.manual_seed(8)
    m=WorldModel(12).eval()
    image=torch.zeros(1,3,128,128); scan=torch.ones(1,12)*.5; mask=torch.ones_like(scan)
    a=m(image,scan,mask,torch.tensor([[0.,0.]]),torch.tensor([.2]))
    b=m(image,scan,mask,torch.tensor([[.1,.2]]),torch.tensor([.4]))
    assert not torch.allclose(a[0],b[0])
    assert not torch.allclose(a[1],b[1])


def test_masked_loss_ignores_unobserved_target():
    pred=torch.tensor([[.2,.8]],requires_grad=True)
    value=masked_scan_mse(pred,torch.tensor([[.2,100.]]),torch.tensor([[1.,0.]]))
    assert float(value.detach())==0
    value.backward()
    assert torch.equal(pred.grad,torch.zeros_like(pred))


def test_vla_step_and_finite_output_classes():
    m=TinyVLA(12,3)
    logits=m(torch.zeros(2,3,128,128),torch.ones(2,12)*.5,torch.ones(2,12),torch.tensor([[2,3,0],[4,5,6]]),torch.zeros(2,3,2))
    assert logits.shape==(2,3)
    loss=torch.nn.functional.cross_entropy(logits,torch.tensor([0,2])); loss.backward()
    assert torch.isfinite(loss)
