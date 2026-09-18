"""mjlab 1.3.0 + warp-lang 1.14 の互換パッチ。

mjlab 1.3.0 の ``Simulation._should_use_cuda_graph`` は ``wp.context.runtime`` を読むが、
warp-lang 1.14 は ``warp.context`` を公開しなくなったので `AttributeError: module 'warp'
has no attribute 'context'` で環境構築が落ちる。CUDA graph は性能機能なので、
driver version ではなく mempool の有無だけで判定するように差し替える
(mempool が使えるドライバなら graph capture も使える、という前提)。

**なぜ venv の sitecustomize.py ではなくここに置くか**: Debian/Ubuntu の system python は
``/usr/lib/python3.12/sitecustomize.py`` を同梱しており、そちらが stdlib 側にあるため
venv の site-packages に置いた sitecustomize.py は import されない(uv が system python を
拾った環境で実際に踏んだ)。タスクパッケージの import 時に当てれば venv の作り方に依存しない。
"""


def apply() -> None:
  """warp が古い API を持たない場合だけパッチする。冪等。"""
  import warp as wp

  if getattr(getattr(wp, "context", None), "runtime", None) is not None:
    return  # warp 側に wp.context.runtime がある = パッチ不要

  from mjlab.sim import sim as _sim

  def _should_use_cuda_graph(self) -> bool:
    return bool(self.wp_device.is_cuda and wp.is_mempool_enabled(self.wp_device))

  _sim.Simulation._should_use_cuda_graph = _should_use_cuda_graph
