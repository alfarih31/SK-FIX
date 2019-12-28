# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
import grpc

import SK_pb2 as SK__pb2


class WSStub(object):
  # missing associated documentation comment in .proto file
  pass

  def __init__(self, channel):
    """Constructor.

    Args:
      channel: A grpc.Channel.
    """
    self.clientStream = channel.unary_unary(
        '/WS/clientStream',
        request_serializer=SK__pb2.fromClient.SerializeToString,
        response_deserializer=SK__pb2.toClient.FromString,
        )


class WSServicer(object):
  # missing associated documentation comment in .proto file
  pass

  def clientStream(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')


def add_WSServicer_to_server(servicer, server):
  rpc_method_handlers = {
      'clientStream': grpc.unary_unary_rpc_method_handler(
          servicer.clientStream,
          request_deserializer=SK__pb2.fromClient.FromString,
          response_serializer=SK__pb2.toClient.SerializeToString,
      ),
  }
  generic_handler = grpc.method_handlers_generic_handler(
      'WS', rpc_method_handlers)
  server.add_generic_rpc_handlers((generic_handler,))
