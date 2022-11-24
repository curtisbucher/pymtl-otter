from pymtl3 import Wire

def signed_lt(A: Wire, B: Wire) -> bool:
    """
    Returns whether $signed(A) < $signed(B)
    """
    A_neg = A[31]
    B_neg = B[31]

    return (
        (A_neg & ~B_neg) | # A is negative, B is positive
        ((~A_neg & ~B_neg) and A < B) # both are positive and A is smaller
    )
