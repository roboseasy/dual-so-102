"""host ↔ client 전송 형식 — 기존 양팔 host 와 같다 (LeKiwi 방식).

- 관측(host → client): JSON 객체 {관절키: float, 카메라이름: base64 JPEG}.
	인코딩에 실패한 프레임은 "" 이다.
- 명령(client → host): JSON 객체 {관절키: float}.

형식을 바꾸면 이미 Pi 에서 도는 기존 host 와 새 client 가 붙지 않는다.
"""

import base64
import binascii

import cv2
import numpy as np


def encode_observation(
	observation: dict[str, object],
	jpeg_quality: int = 90,
) -> dict[str, float | str]:
	"""관측을 전송 형식으로 바꾼다.

	Args:
		observation: 로봇 관측 (관절키 → 숫자, 카메라이름 → HxWx3 배열,
			이번에 못 읽은 프레임은 None).
		jpeg_quality: JPEG 품질 (기존 host 와 같은 90).

	Returns:
		관절값은 float, 프레임은 base64 JPEG 문자열(없거나 실패하면 "")인 dict.
	"""
	encoded: dict[str, float | str] = {}
	for key, value in observation.items():
		if value is None:
			encoded[key] = ''
		elif isinstance(value, np.ndarray):
			is_ok, buffer = cv2.imencode('.jpg', value,
				[int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
			encoded[key] = (base64.b64encode(buffer).decode('ascii')
				if is_ok else '')
		else:
			encoded[key] = float(value)
	return encoded


def decode_frame(image_b64: object) -> np.ndarray | None:
	"""전송된 프레임 하나를 HxWx3 배열로 되돌린다.

	Args:
		image_b64: base64 JPEG 문자열 (네트워크에서 온 값이라 무엇이든 올 수 있다).

	Returns:
		프레임. 비었거나 깨졌으면 None.
	"""
	if not isinstance(image_b64, str) or not image_b64:
		return None
	try:
		jpeg = base64.b64decode(image_b64, validate=True)
		frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8),
			cv2.IMREAD_COLOR)
	except (binascii.Error, ValueError, cv2.error):
		return None
	return frame
